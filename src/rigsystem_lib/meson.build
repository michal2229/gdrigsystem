project('rigsystem_lib', 'cpp',
  version : '0.1',
  default_options : ['warning_level=3', 'cpp_std=c++20', 'werror=false'] )


#if get_option('buildtype') == 'fast'
add_project_arguments('-O3', '-ffast-math', '-g3', language : 'cpp')
#endif

#libomp = dependency('openmp')


# building the lib
librigsystem_incdir = include_directories(['src'])
librigsystem_src = [ 'src/rigsystem_common.cpp' ]
librigsystem = library('rigsystem', librigsystem_src, version : '0.1.0', soversion : '1')  #, dependencies: [ libomp ] )


# building dependencies libs
libasciigraphics_incdir = include_directories([ 'src_app/deps/ascii-graphics/src' ])
libasciigraphics_src = [
    'src_app/deps/ascii-graphics/src/agm.cpp',
    'src_app/deps/ascii-graphics/src/Camera.cpp',
    'src_app/deps/ascii-graphics/src/Lights.cpp',
    'src_app/deps/ascii-graphics/src/Mesh.cpp',
    'src_app/deps/ascii-graphics/src/Screen.cpp'
]
libasciigraphics = library('asciigraphics', libasciigraphics_src, override_options : ['werror=false', 'no-everything=true'])

libasciidrawrigsystem_incdir = include_directories([ 'src_app/deps/ascii-draw-rigsystem' ])
libasciidrawrigsystem_src = [ 'src_app/deps/ascii-draw-rigsystem/ascii_draw_rigsystem.cpp' ]
libasciidrawrigsystem = library('asciidrawrigsystem', 
                                      libasciidrawrigsystem_src, 
                                      include_directories : [librigsystem_incdir, libasciigraphics_incdir, libasciidrawrigsystem_incdir], 
                                      link_with : [librigsystem, libasciigraphics])


# building sample app
src_app_test = [ 'src_app/app_test.cpp' ]
app_test = executable('app_test', 
                  src_app_test, 
                  include_directories : [librigsystem_incdir, libasciigraphics_incdir, libasciidrawrigsystem_incdir], 
                  link_with : [librigsystem, libasciigraphics, libasciidrawrigsystem])

