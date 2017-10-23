;;; -*- Mode: LISP; Package: CL-USER -*-
;;;
;;; Copyright (C) Peter Van Eynde 2001 and Kevin Rosenberg 2002-2003
;;;
;;; License: LGPL v2
;;;
(in-package "COMMON-LISP-USER")

(handler-case
    (load "/usr/share/common-lisp/source/common-lisp-controller/common-lisp-controller.lisp")
  (error (e)
    (format t "~%Error during load of common-lisp-controller.lisp: ~A~%" e)
    (sb-unix:unix-exit 1)))

(handler-case
    (common-lisp-controller:init-common-lisp-controller-v4 "sbcl")
  (error (e)
    (format t "~%Error running init-common-lisp-controller-v4: ~A~%" e)
    (sb-unix:unix-exit 1)))

(when (probe-file #p"/etc/lisp.config")
  (load #p"/etc/lisp.config"))

(setf (logical-pathname-translations "SYS")
      '(("SYS:**;*.*.*"
         #P"/usr/share/sbcl-source/**/*.*")))

(set-dispatch-macro-character #\# #\!
  (lambda (stream bang arg)
    (declare (ignore bang arg))
    (read-line stream)
    (values)))

(ignore-errors
 (format t "~%Saving to sbcl-new.core...")
 (sb-ext:gc :full t)
 (sb-ext:save-lisp-and-die "sbcl-new.core"))
