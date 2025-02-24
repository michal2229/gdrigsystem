project('rigsystem_lib', 'cpp',
  version : '0.1',
  default_options : ['warning_level=3',
                     'cpp_std=c++20'])


# building the lib

src_lib = [
    'src/rigsystem_common.cpp'
]

librigsystem = library('rigsystem', src_lib, version : '0.1.0', soversion : '1')


# building sample app

librigsystem_incdir = include_directories(['src', 'src_app/deps/ascii-graphics/src'])

src_app_test = [
    'src_app/app_test.cpp',
    
    'src_app/deps/ascii-graphics/src/agm.cpp',
    'src_app/deps/ascii-graphics/src/Camera.cpp',
    'src_app/deps/ascii-graphics/src/Lights.cpp',
    'src_app/deps/ascii-graphics/src/Mesh.cpp',
    'src_app/deps/ascii-graphics/src/Screen.cpp',
]

app_test = executable('app_test', src_app_test, include_directories : librigsystem_incdir, link_with : librigsystem)

