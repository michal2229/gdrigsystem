project(
  'rigsystem_lib',
  'cpp',
  version: '0.1',
  default_options: ['warning_level=3', 'cpp_std=c++20', 'werror=false'],
)

#if get_option('buildtype') == 'fast'
ccld_flags = ['-fno-exceptions', '-O3', '-ffast-math', '-mavx2', '-g3', '-fno-omit-frame-pointer' ]
add_project_arguments(ccld_flags, language: 'cpp')
add_project_link_arguments(ccld_flags, language: 'cpp')
#endif


# building library dependencies
depyamlcpp_sub = subproject('yaml-cpp')
depyamlcpp_incdir = include_directories(['subprojects/yaml-cpp/include'])
depyamlcpp = depyamlcpp_sub.get_variable('yaml_cpp_dep')


# building the lib
librigsystem_src = ['src/rigsystem_common.cpp']
librigsystem_incdir = include_directories(['src'])
librigsystem = library(
  'rigsystem',
  librigsystem_src,
  include_directories: [depyamlcpp_incdir],
  dependencies: [depyamlcpp],
  version: '0.1.1',
  soversion: '1',
)


# building app dependencies libs
libasciigraphics_incdir = include_directories(['src_app/deps/ascii-graphics/src'])
libasciigraphics_src = [
  'src_app/deps/ascii-graphics/src/agm.cpp',
  'src_app/deps/ascii-graphics/src/Camera.cpp',
  'src_app/deps/ascii-graphics/src/Lights.cpp',
  'src_app/deps/ascii-graphics/src/Mesh.cpp',
  'src_app/deps/ascii-graphics/src/Screen.cpp',
]
libasciigraphics = library(
  'asciigraphics',
  libasciigraphics_src,
  override_options: ['werror=false', 'no-everything=true'],
)

libasciidrawrigsystem_incdir = include_directories(['src_app/deps/ascii-draw-rigsystem'])
libasciidrawrigsystem_src = ['src_app/deps/ascii-draw-rigsystem/ascii_draw_rigsystem.cpp']
libasciidrawrigsystem = library(
  'asciidrawrigsystem',
  libasciidrawrigsystem_src,
  include_directories: [librigsystem_incdir, libasciigraphics_incdir, libasciidrawrigsystem_incdir],
  link_with: [librigsystem, libasciigraphics],
)


# building sample app
src_app_test = ['src_app/app_test.cpp']
app_test = executable(
  'app_test',
  src_app_test,
  include_directories: [librigsystem_incdir, libasciigraphics_incdir, libasciidrawrigsystem_incdir],
  link_with: [librigsystem, libasciigraphics, libasciidrawrigsystem],
)
