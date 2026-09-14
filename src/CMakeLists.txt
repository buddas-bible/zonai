add_library(zonai INTERFACE)
add_library(zonai::zonai ALIAS zonai)

target_compile_features(zonai INTERFACE cxx_std_20)
