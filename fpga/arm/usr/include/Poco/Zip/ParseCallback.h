//
// ParseCallback.h
//
// $Id: //poco/1.3/Zip/include/Poco/Zip/ParseCallback.h#3 $
//
// Library: Zip
// Package: Zip
// Module:  ParseCallback
//
// Definition of the ParseCallback class.
//
// Copyright (c) 2007, Applied Informatics Software Engineering GmbH.
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


#ifndef Zip_ParseCallback_INCLUDED
#define Zip_ParseCallback_INCLUDED


#include "Poco/Zip/Zip.h"
#include <istream>


namespace Poco {
namespace Zip {


class ZipLocalFileHeader;


class Zip_API ParseCallback
	/// Interface for callbacks to handle ZipData
{
public:
	ParseCallback();
		/// Creates the ParseCallback.

	virtual ~ParseCallback();
		/// Destroys the ParseCallback.

	virtual bool handleZipEntry(std::istream& zipStream, const ZipLocalFileHeader& hdr) = 0;
		/// Handles parsing of the data of a single Zip Entry. zipStream is guaranteed to be at the very first data byte.
		/// Note that a callback class SHOULD consume all data inside a zip file, ie. after
		/// processing the next 4 bytes point the next ZipLocalFileHeader or the ZipDirectory.
		/// If it fails to do so, it must return false, otherwise true.

};


} } // namespace Poco::Zip


#endif // Zip_ParseCallback_INCLUDED
