/***********************************************************************
StringMarshaller - Helper class to serialize/unserialize C and C++
standard strings to/from storage classes that support typed
(templatized) read and write methods.
Copyright (c) 2008-2020 Oliver Kreylos

This file is part of the Miscellaneous Support Library (Misc).

The Miscellaneous Support Library is free software; you can
redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version.

The Miscellaneous Support Library is distributed in the hope that it
will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with the Miscellaneous Support Library; if not, write to the Free
Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
02111-1307 USA
***********************************************************************/

#ifndef MISC_STRINGMARSHALLER_INCLUDED
#define MISC_STRINGMARSHALLER_INCLUDED

#include <string.h>
#include <string>
#include <Misc/SizedTypes.h>

namespace Misc {

template <class PipeParam>
inline
void
writeCString(
	const char* string,
	PipeParam& pipe) // Writes a C string to a pipe
	{
	if(string!=0)
		{
		size_t length=strlen(string);
		pipe.template write<Misc::UInt32>(Misc::UInt32(length)); // Write the string's length
		pipe.template write<char>(string,length); // Write the string without its terminating NUL character
		}
	else
		pipe.template write<Misc::UInt32>(~Misc::UInt32(0)); // Write a magic number for null-pointer strings
	}

template <class PipeParam>
inline
void
writeCppString(
	const std::string& string,
	PipeParam& pipe) // Writes a C++ string to a pipe
	{
	size_t length=string.length();
	pipe.template write<Misc::UInt32>(Misc::UInt32(length)); // Write the string's length
	pipe.template write<char>(string.data(),length); // Write the string without its terminating NUL character
	}

template <class PipeParam>
inline
char*
readCString(
	PipeParam& pipe) // Reads a C string from a pipe; returns a new[]-allocated character array
	{
	/* Read the string's length: */
	size_t length=pipe.template read<Misc::UInt32>();
	
	if(length!=~Misc::UInt32(0)) // Look for the null-pointer string magic number
		{
		/* Allocate the character array and read the characters: */
		char* result=new char[length+1];
		pipe.template read<char>(result,length);
		
		/* NUL-terminate and return the string: */
		result[length]='\0';
		return result;
		}
	else
		return 0;
	}

template <class PipeParam>
inline
std::string
readCppString(
	PipeParam& pipe) // Reads a C++ string from a pipe
	{
	/* Read the string's length: */
	size_t length=pipe.template read<Misc::UInt32>();
	
	if(length!=~Misc::UInt32(0)) // Look for the null-pointer string magic number
		{
		/* Read the string in chunks (unfortunately, there is no API to read directly into the std::string): */
		std::string result;
		result.reserve(length);
		while(length>0)
			{
			std::string::value_type buffer[256];
			size_t readLength=length;
			if(readLength>sizeof(buffer))
				readLength=sizeof(buffer);
			pipe.template read<std::string::value_type>(buffer,readLength);
			result.append(buffer,readLength);
			length-=readLength;
			}
		
		/* Return the string: */
		return result;
		}
	else
		return std::string(); // Can't return a null pointer; so return an empty string instead
	}

}

#endif
