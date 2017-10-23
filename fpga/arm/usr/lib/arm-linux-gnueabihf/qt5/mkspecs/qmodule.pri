CONFIG +=  compile_examples system-sqlite qpa largefile nostrip precompile_header use_gold_linker enable_new_dtags
QT_BUILD_PARTS += libs tools examples
QT_NO_DEFINES =  IMAGEFORMAT_JPEG OPENVG TABLET TSLIB XINPUT ZLIB
QT_QCONFIG_PATH = 
host_build {
    QT_CPU_FEATURES.arm = 
} else {
    QT_CPU_FEATURES.arm = 
}
QT_COORD_TYPE = double
QT_CFLAGS_PSQL   = -I/usr/include/postgresql
QT_CFLAGS_MYSQL   = -I/usr/include/mysql
QT_LFLAGS_MYSQL   = -lmysqlclient -lpthread -lz -lm -lrt -ldl
QT_LFLAGS_SQLITE   = -lsqlite3
QT_LFLAGS_ODBC   = -lodbc
QMAKE_CFLAGS = -g -O2 -fstack-protector-strong -Wformat -Werror=format-security -Wdate-time -D_FORTIFY_SOURCE=2
QMAKE_CXXFLAGS = -g -O2 -fstack-protector-strong -Wformat -Werror=format-security -Wdate-time -D_FORTIFY_SOURCE=2
QMAKE_LFLAGS = -Wl,-Bsymbolic-functions -Wl,-z,relro -Wl,--as-needed
styles += mac fusion windows
QT_LIBS_DBUS = -ldbus-1
QT_CFLAGS_DBUS = -I/usr/include/dbus-1.0 -I/usr/lib/arm-linux-gnueabihf/dbus-1.0/include
QT_HOST_CFLAGS_DBUS = -I/usr/include/dbus-1.0 -I/usr/lib/arm-linux-gnueabihf/dbus-1.0/include
QT_CFLAGS_GLIB = -pthread -I/usr/include/glib-2.0 -I/usr/lib/arm-linux-gnueabihf/glib-2.0/include
QT_LIBS_GLIB = -lgthread-2.0 -pthread -lglib-2.0
QT_CFLAGS_QGTKSTYLE = -pthread -I/usr/include/gtk-2.0 -I/usr/lib/arm-linux-gnueabihf/gtk-2.0/include -I/usr/include/gio-unix-2.0/ -I/usr/include/cairo -I/usr/include/pango-1.0 -I/usr/include/cairo -I/usr/include/pixman-1 -I/usr/include/libpng12 -I/usr/include/gdk-pixbuf-2.0 -I/usr/include/libpng12 -I/usr/include/pango-1.0 -I/usr/include/harfbuzz -I/usr/include/pango-1.0 -I/usr/include/freetype2 -I/usr/include/atk-1.0 -I/usr/include/glib-2.0 -I/usr/lib/arm-linux-gnueabihf/glib-2.0/include
QT_LIBS_QGTKSTYLE = -lgobject-2.0 -lglib-2.0
QT_CFLAGS_QGTK2 = -pthread -I/usr/include/gtk-2.0 -I/usr/lib/arm-linux-gnueabihf/gtk-2.0/include -I/usr/include/gio-unix-2.0/ -I/usr/include/cairo -I/usr/include/pango-1.0 -I/usr/include/cairo -I/usr/include/pixman-1 -I/usr/include/libpng12 -I/usr/include/gdk-pixbuf-2.0 -I/usr/include/libpng12 -I/usr/include/pango-1.0 -I/usr/include/harfbuzz -I/usr/include/pango-1.0 -I/usr/include/freetype2 -I/usr/include/atk-1.0 -I/usr/include/glib-2.0 -I/usr/lib/arm-linux-gnueabihf/glib-2.0/include
QT_LIBS_QGTK2 = -lgtk-x11-2.0 -lgdk-x11-2.0 -lpangocairo-1.0 -latk-1.0 -lcairo -lgdk_pixbuf-2.0 -lgio-2.0 -lpangoft2-1.0 -lpango-1.0 -lgobject-2.0 -lglib-2.0 -lfontconfig -lfreetype
QT_CFLAGS_PULSEAUDIO = -D_REENTRANT -I/usr/include/glib-2.0 -I/usr/lib/arm-linux-gnueabihf/glib-2.0/include
QT_LIBS_PULSEAUDIO = -lpulse-mainloop-glib -lpulse -lglib-2.0
QMAKE_INCDIR_OPENGL_ES2 = 
QMAKE_LIBDIR_OPENGL_ES2 = 
QMAKE_LIBS_OPENGL_ES2 =  "-lGLESv2"
QMAKE_CFLAGS_OPENGL_ES2 = 
QMAKE_CFLAGS_FONTCONFIG = -I/usr/include/freetype2
QMAKE_LIBS_FONTCONFIG = -lfontconfig -lfreetype
QMAKE_INCDIR_LIBUDEV = 
QMAKE_LIBS_LIBUDEV = -ludev
DEFINES += QT_NO_TSLIB
QMAKE_INCDIR_XKBCOMMON_EVDEV = 
QMAKE_LIBS_XKBCOMMON_EVDEV = -lxkbcommon
QMAKE_LIBINPUT_VERSION_MAJOR = 1
QMAKE_LIBINPUT_VERSION_MINOR = 2
QMAKE_INCDIR_LIBINPUT = 
QMAKE_LIBS_LIBINPUT = -linput
QMAKE_LIBXI_VERSION_MAJOR = 1
QMAKE_LIBXI_VERSION_MINOR = 7
QMAKE_LIBXI_VERSION_PATCH = 6
QMAKE_X11_PREFIX = /usr
QMAKE_CFLAGS_XKBCOMMON = 
QMAKE_LIBS_XKBCOMMON = -lxkbcommon-x11 -lxkbcommon
QMAKE_INCDIR_EGL = /usr/include/libdrm
QMAKE_LIBS_EGL = -lEGL
QMAKE_CFLAGS_EGL = 
QMAKE_CFLAGS_XCB = 
QMAKE_LIBS_XCB = -lxcb-sync -lxcb-xfixes -lxcb-randr -lxcb-render -lxcb-image -lxcb-shm -lxcb-keysyms -lxcb-icccm -lxcb-shape -lxcb
sql-drivers = 
sql-plugins =  mysql odbc psql sqlite tds
