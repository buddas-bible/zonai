#pragma once

#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "world_state_snapshot.h"

namespace zonai::determinism::test
{

enum class replayCommandType : std::uint8_t
{
    forceToCenter = 1,
    impulseToCenter = 2,
    setLinearVelocity = 3,
    setGravity = 4
};

struct replayCommand
{
    std::uint32_t tick = 0;
    replayCommandType type = replayCommandType::forceToCenter;
    std::uint8_t bodySlot = 0;
    float x = 0.0f;
    float y = 0.0f;
};

struct replayInput
{
    std::uint32_t tickCount = 60;
    float timeStep = 1.0f / 60.0f;
    std::uint32_t substepCount = 4;
    std::vector<replayCommand> commands;
};

struct replayArchive
{
    std::vector<std::uint8_t> inputBytes;
    std::vector<stateSnapshot> frames;
};

struct replayMismatch
{
    std::uint32_t tick = 0;
    snapshotDifference difference{};
};

class replayByteWriter
{
public:
    void writeU8( std::uint8_t value ) { bytes_.push_back( value ); }

    void writeBits( std::uint64_t value, std::uint8_t width )
    {
        for( std::uint8_t i = 0; i < width; ++i )
            writeU8( static_cast<std::uint8_t>( value >> ( 8 * i ) ) );
    }

    void writeU16( std::uint16_t value ) { writeBits( value, 2 ); }
    void writeU32( std::uint32_t value ) { writeBits( value, 4 ); }

    void writeFloat( float value )
    {
        static_assert( sizeof( float ) == sizeof( std::uint32_t ) );
        writeU32( std::bit_cast<std::uint32_t>( value ) );
    }

    void writeBytes( std::span<const std::uint8_t> value )
    {
        bytes_.insert( bytes_.end(), value.begin(), value.end() );
    }

    [[nodiscard]] std::vector<std::uint8_t> finish() && { return std::move( bytes_ ); }

private:
    std::vector<std::uint8_t> bytes_;
};

class replayByteReader
{
public:
    explicit replayByteReader( std::span<const std::uint8_t> bytes ) : bytes_( bytes ) {}

    [[nodiscard]] bool finished() const noexcept { return offset_ == bytes_.size(); }

    [[nodiscard]] std::uint64_t readBits( std::uint8_t width )
    {
        if( width == 0 || width > 8 || width > bytes_.size() - offset_ )
            throw std::runtime_error( "truncated or invalid replay record" );

        std::uint64_t value = 0;

        for( std::uint8_t i = 0; i < width; ++i )
            value |= static_cast<std::uint64_t>( bytes_[offset_++] ) << ( 8 * i );

        return value;
    }

    [[nodiscard]] std::uint8_t readU8() { return static_cast<std::uint8_t>( readBits( 1 ) ); }
    [[nodiscard]] std::uint16_t readU16() { return static_cast<std::uint16_t>( readBits( 2 ) ); }
    [[nodiscard]] std::uint32_t readU32() { return static_cast<std::uint32_t>( readBits( 4 ) ); }

    [[nodiscard]] std::vector<std::uint8_t> readBytes( std::size_t count )
    {
        if( count > bytes_.size() - offset_ )
            throw std::runtime_error( "truncated replay byte array" );

        std::vector<std::uint8_t> result( bytes_.begin() + offset_,
            bytes_.begin() + offset_ + count );
        offset_ += count;
        return result;
    }

    void requireEnd() const
    {
        if( !finished() ) throw std::runtime_error( "trailing replay bytes" );
    }

private:
    std::span<const std::uint8_t> bytes_;
    std::size_t offset_ = 0;
};

inline void validateReplayInput( const replayInput& input )
{
    if( input.tickCount == 0 || input.tickCount > 10000 ||
        input.substepCount == 0 || input.substepCount > 64 ||
        !std::isfinite( input.timeStep ) || input.timeStep <= 0.0f ||
        input.commands.size() > 100000 )
        throw std::invalid_argument( "invalid replay configuration" );

    std::uint32_t previousTick = 0;

    for( const replayCommand& command : input.commands )
    {
        const std::uint8_t kind = static_cast<std::uint8_t>( command.type );

        if( command.tick == 0 || command.tick > input.tickCount ||
            command.tick < previousTick || kind < 1 || kind > 4 ||
            ( kind == 4 ? command.bodySlot != 0 :
                ( command.bodySlot != 1 && command.bodySlot != 2 ) ) ||
            !std::isfinite( command.x ) || !std::isfinite( command.y ) )
            throw std::invalid_argument( "invalid or unsorted Tick input command" );

        previousTick = command.tick;
    }
}

// Canonical input log. Same-tick commands retain their original insertion order.
[[nodiscard]] inline std::vector<std::uint8_t> encodeReplayInput( const replayInput& input )
{
    validateReplayInput( input );

    replayByteWriter writer{};
    writer.writeU32( 0x314e495au ); // ZIN1
    writer.writeU16( 1 );
    writer.writeU32( input.tickCount );
    writer.writeFloat( input.timeStep );
    writer.writeU32( input.substepCount );
    writer.writeU32( static_cast<std::uint32_t>( input.commands.size() ) );

    for( const replayCommand& command : input.commands )
    {
        writer.writeU32( command.tick );
        writer.writeU8( static_cast<std::uint8_t>( command.type ) );
        writer.writeU8( command.bodySlot );
        writer.writeFloat( command.x );
        writer.writeFloat( command.y );
    }

    return std::move( writer ).finish();
}

[[nodiscard]] inline replayInput decodeReplayInput( std::span<const std::uint8_t> bytes )
{
    replayByteReader reader( bytes );

    if( reader.readU32() != 0x314e495au || reader.readU16() != 1 )
        throw std::runtime_error( "unknown replay input format" );

    replayInput input{};
    input.tickCount = reader.readU32();
    input.timeStep = std::bit_cast<float>( reader.readU32() );
    input.substepCount = reader.readU32();
    const std::uint32_t count = reader.readU32();

    if( count > 100000 ) throw std::runtime_error( "too many replay commands" );
    input.commands.reserve( count );

    for( std::uint32_t i = 0; i < count; ++i )
    {
        replayCommand command{};
        command.tick = reader.readU32();
        command.type = static_cast<replayCommandType>( reader.readU8() );
        command.bodySlot = reader.readU8();
        command.x = std::bit_cast<float>( reader.readU32() );
        command.y = std::bit_cast<float>( reader.readU32() );
        input.commands.push_back( command );
    }

    reader.requireEnd();
    validateReplayInput( input );
    return input;
}

[[nodiscard]] inline stateSnapshot decodeStateSnapshot( std::span<const std::uint8_t> bytes )
{
    replayByteReader reader( bytes );
    stateSnapshot result{};
    result.bytes.assign( bytes.begin(), bytes.end() );

    while( !reader.finished() )
    {
        const std::uint16_t nameLength = reader.readU16();

        if( nameLength == 0 || nameLength > 2048 )
            throw std::runtime_error( "invalid snapshot field name length" );

        const std::vector<std::uint8_t> name = reader.readBytes( nameLength );
        const std::uint8_t width = reader.readU8();

        if( width != 1 && width != 2 && width != 4 && width != 8 )
            throw std::runtime_error( "invalid snapshot field width" );

        std::string fieldName( name.begin(), name.end() );
        const std::uint64_t bits = reader.readBits( width );
        result.fields.push_back( { std::move( fieldName ), bits, width } );
    }

    if( result.fields.empty() )
        throw std::runtime_error( "snapshot has no fields" );

    return result;
}

// Trace embeds the full command log, so an exported archive can be replayed
// without relying on a separately stored mutable input file.
[[nodiscard]] inline std::vector<std::uint8_t> encodeReplayArchive( const replayArchive& archive )
{
    const replayInput input = decodeReplayInput( archive.inputBytes );

    if( archive.frames.size() != static_cast<std::size_t>( input.tickCount ) + 1 )
        throw std::invalid_argument( "replay archive Tick count mismatch" );

    replayByteWriter writer{};
    writer.writeU32( 0x3152545au ); // ZTR1
    writer.writeU16( 1 );
    writer.writeU32( static_cast<std::uint32_t>( archive.inputBytes.size() ) );
    writer.writeBytes( archive.inputBytes );
    writer.writeU32( static_cast<std::uint32_t>( archive.frames.size() ) );

    for( const stateSnapshot& snapshot : archive.frames )
    {
        if( snapshot.bytes.empty() || snapshot.bytes.size() > 4 * 1024 * 1024 )
            throw std::invalid_argument( "invalid snapshot frame size" );

        writer.writeU32( static_cast<std::uint32_t>( snapshot.bytes.size() ) );
        writer.writeBytes( snapshot.bytes );
    }

    return std::move( writer ).finish();
}

[[nodiscard]] inline replayArchive decodeReplayArchive( std::span<const std::uint8_t> bytes )
{
    replayByteReader reader( bytes );

    if( reader.readU32() != 0x3152545au || reader.readU16() != 1 )
        throw std::runtime_error( "unknown replay archive format" );

    const std::uint32_t inputLength = reader.readU32();
    if( inputLength > 4 * 1024 * 1024 )
        throw std::runtime_error( "replay input too large" );

    replayArchive result{};
    result.inputBytes = reader.readBytes( inputLength );

    const replayInput input = decodeReplayInput( result.inputBytes );
    const std::uint32_t frameCount = reader.readU32();

    if( frameCount != input.tickCount + 1 )
        throw std::runtime_error( "invalid replay archive frame count" );

    result.frames.reserve( frameCount );

    for( std::uint32_t i = 0; i < frameCount; ++i )
    {
        const std::uint32_t length = reader.readU32();

        if( length == 0 || length > 4 * 1024 * 1024 )
            throw std::runtime_error( "invalid replay snapshot length" );

        const auto snapshotBytes = reader.readBytes( length );
        result.frames.push_back( decodeStateSnapshot( snapshotBytes ) );
    }

    reader.requireEnd();
    return result;
}

[[nodiscard]] inline std::optional<replayMismatch> firstReplayDifference(
    const replayArchive& left, const replayArchive& right )
{
    if( left.inputBytes != right.inputBytes )
    {
        const std::size_t size = std::min( left.inputBytes.size(), right.inputBytes.size() );
        std::size_t i = 0;

        while( i < size && left.inputBytes[i] == right.inputBytes[i] ) ++i;

        return replayMismatch{
            0,
            { "input.byte[" + std::to_string( i ) + "]",
                i < left.inputBytes.size() ? left.inputBytes[i] : 0,
                i < right.inputBytes.size() ? right.inputBytes[i] : 0,
                static_cast<std::uint8_t>( i < left.inputBytes.size() ? 1 : 0 ),
                static_cast<std::uint8_t>( i < right.inputBytes.size() ? 1 : 0 ) }
        };
    }

    if( left.frames.size() != right.frames.size() )
        return replayMismatch{ 0, { "trace.frameCount",
            left.frames.size(), right.frames.size(), 8, 8 } };

    for( std::size_t i = 0; i < left.frames.size(); ++i )
    {
        if( const auto difference = firstDifference( left.frames[i], right.frames[i] ) )
            return replayMismatch{ static_cast<std::uint32_t>( i ), *difference };
    }

    return std::nullopt;
}

inline void writeReplayFile( const std::string& path, std::span<const std::uint8_t> bytes )
{
    std::ofstream stream( path, std::ios::binary | std::ios::trunc );
    if( !stream ) throw std::runtime_error( "cannot create replay file: " + path );

    stream.write( reinterpret_cast<const char*>( bytes.data() ),
        static_cast<std::streamsize>( bytes.size() ) );

    if( !stream ) throw std::runtime_error( "cannot write replay file: " + path );
}

[[nodiscard]] inline std::vector<std::uint8_t> readReplayFile( const std::string& path )
{
    std::ifstream stream( path, std::ios::binary | std::ios::ate );
    if( !stream ) throw std::runtime_error( "cannot open replay file: " + path );

    const std::streamoff length = stream.tellg();

    if( length < 0 || length > 64 * 1024 * 1024 )
        throw std::runtime_error( "invalid replay file size: " + path );

    std::vector<std::uint8_t> bytes( static_cast<std::size_t>( length ) );
    stream.seekg( 0, std::ios::beg );
    stream.read( reinterpret_cast<char*>( bytes.data() ), length );

    if( !stream ) throw std::runtime_error( "cannot read replay file: " + path );

    return bytes;
}

} // namespace zonai::determinism::test
