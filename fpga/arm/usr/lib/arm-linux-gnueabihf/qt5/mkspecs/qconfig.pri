#configuration
CONFIG +=  shared qpa no_mocdepend release qt_no_framework
host_build {
    QT_ARCH = arm
    QT_TARGET_ARCH = arm
} else {
    QT_ARCH = arm
    QMAKE_DEFAULT_LIBDIRS = /usr/lib /usr/lib/gcc/arm-linux-gnueabihf/5 /usr/lib/arm-linux-gnueabihf /lib/arm-linux-gnueabihf /lib
    QMAKE_DEFAULT_INCDIRS = /usr/include/c++/5 /usr/include/arm-linux-gnueabihf/c++/5 /usr/include/c++/5/backward /usr/lib/gcc/arm-linux-gnueabihf/5/include /usr/local/include /usr/lib/gcc/arm-linux-gnueabihf/5/include-fixed /usr/include/arm-linux-gnueabihf /usr/include
}
QT_CONFIG +=  minimal-config small-config medium-config large-config full-config gtk2 gtkstyle fontconfig libudev evdev xkbcommon-evdev libinput xlib xrender xcb-plugin xcb-render xcb-glx xcb-xlib xcb-sm accessibility-atspi-bridge linuxfb kms c++11 accessibility egl egl_x11 eglfs opengl opengles2 shared qpa reduce_exports clock-gettime clock-monotonic posix_fallocate mremap getaddrinfo ipv6ifname getifaddrs inotify eventfd system-jpeg system-png png system-freetype system-harfbuzz system-zlib mtdev nis cups iconv glib dbus dbus-linked openssl libproxy xcb xinput2 alsa pulseaudio gstreamer-1.0 icu concurrent audio-backend release

#versioning
QT_VERSION = 5.5.1
QT_MAJOR_VERSION = 5
QT_MINOR_VERSION = 5
QT_PATCH_VERSION = 1

#namespaces
QT_LIBINFIX = 
QT_NAMESPACE = 

QT_EDITION = OpenSource

QT_GCC_MAJOR_VERSION = 5
QT_GCC_MINOR_VERSION = 4
QT_GCC_PATCH_VERSION = 0
