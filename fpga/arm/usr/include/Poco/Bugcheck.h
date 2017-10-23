//
// Bugcheck.h
//
// $Id: //poco/1.3/Foundation/include/Poco/Bugcheck.h#3 $
//
// Library: Foundation
// Package: Core
// Module:  Bugcheck
//
// Definition of the Bugcheck class and the self-testing macros.
//
// Copyright (c) 2004-2006, Applied Informatics Software Engineering GmbH.
// and Contributors.
//
// Permission is hereby granted, free of charge, to any person or organization
// obtaining a copy of the software and accompanying documentation covered by
// this license (the "Software") to use, reproduce, display, distribute,
// execute, and transmit the Software, and to prepare derivative works of the
// Software, and to permit third-parties to whom the Software is furnished to
// do so, all subject to the following:
// 
// The copyright notices in the Software and this entire statement, including
// the above license grant, this restriction and the following disclaimer,
// must be included in all copies of the Software, in whole or in part, and
// all derivative works of the Software, unless such copies or derivative
// works are solely in the form of machine-executable object code generated by
// a source language processor.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
// SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
// FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//


#ifndef Foundation_Bugcheck_INCLUDED
#define Foundation_Bugcheck_INCLUDED


#include "Poco/Foundation.h"
#include <string>
#if defined(_DEBUG)
#	include <iostream>
#endif


namespace Poco {


class Foundation_API Bugcheck
	/// This class provides some static methods that are
	/// used by the
	/// poco_assert_dbg(), poco_assert(), poco_check_ptr() 
	/// and poco_bugcheck() macros. 
	/// You should not invoke these methods
	/// directly. Use the macros instead, as they
	/// automatically provide useful context information.
{
public:
	static void assertion(const char* cond, const char* file, int line);
		/// An assertion failed. Break into the debugger, if
		/// possible, then throw an AssertionViolationException.
		
	static void nullPointer(const char* ptr, const char* file, int line);
		/// An null pointer was encountered. Break into the debugger, if
		/// possible, then throw an NullPointerException.

	static void bugcheck(const char* file, int line);
		/// An internal error was encountered. Break into the debugger, if
		/// possible, then throw an BugcheckException.

	static void bugcheck(const char* msg, const char* file, int line);
		/// An internal error was encountered. Break into the debugger, if
		/// possible, then throw an BugcheckException.

	static void debugger(const char* file, int line);
		/// An internal error was encountered. Break into the debugger, if
		/// possible.

	static void debugger(const char* msg, const char* file, int line);
		/// An internal error was encountered. Break into the debugger, if
		/// possible.

protected:
	static std::string what(const char* msg, const char* file, int line);
};


} // namespace Poco


//
// useful macros (these automatically supply line number and file name)
//
#if defined(_DEBUG)
	#define poco_assert_dbg(cond) \
		if (!(cond)) Poco::Bugcheck::assertion(#cond, __FILE__, __LINE__); else (void) 0
#else
	#define poco_assert_dbg(cond)
#endif


#define poco_assert(cond) \
	if (!(cond)) Poco::Bugcheck::assertion(#cond, __FILE__, __LINE__); else (void) 0


#define poco_check_ptr(ptr) \
	if (!(ptr)) Poco::Bugcheck::nullPointer(#ptr, __FILE__, __LINE__); else (void) 0


#define poco_bugcheck() \
	Poco::Bugcheck::bugcheck(__FILE__, __LINE__)


#define poco_bugcheck_msg(msg) \
	Poco::Bugcheck::bugcheck(msg, __FILE__, __LINE__)


#define poco_debugger() \
	Poco::Bugcheck::debugger(__FILE__, __LINE__)


#define poco_debugger_msg(msg) \
	Poco::Bugcheck::debugger(msg, __FILE__, __LINE__)


#if defined(_DEBUG)
#	define poco_stdout_dbg(outstr) \
	std::cout << __FILE__ << '(' << std::dec << __LINE__ << "):" << outstr << std::endl;
#else
#	define poco_stdout_dbg(outstr)
#endif


#if defined(_DEBUG)
#	define poco_stderr_dbg(outstr) \
		std::cerr << __FILE__ << '(' << std::dec << __LINE__ << "):" << outstr << std::endl;
#else
#	define poco_stderr_dbg(outstr)
#endif


//
// poco_static_assert
// 
// The following was ported from <boost/static_assert.hpp>
//


template <bool x>
struct POCO_STATIC_ASSERTION_FAILURE;


template <> 
struct POCO_STATIC_ASSERTION_FAILURE<true> 
{
	enum 
	{ 
		value = 1 
	}; 
};


template <int x> 
struct poco_static_assert_test
{
};


#if defined(__GNUC__) && (__GNUC__ == 3) && ((__GNUC_MINOR__ == 3) || (__GNUC_MINOR__ == 4))
#define poco_static_assert(B) \
	typedef char POCO_JOIN(poco_static_assert_typedef_, __LINE__) \
        [POCO_STATIC_ASSERTION_FAILURE<(bool) (B)>::value]
#else
#define poco_static_assert(B) \
	typedef poco_static_assert_test<sizeof(POCO_STATIC_ASSERTION_FAILURE<(bool) (B)>)> \
		POCO_JOIN(poco_static_assert_typedef_, __LINE__)
#endif


#endif // Foundation_Bugcheck_INCLUDED
