/***********************************************************************
ConfigurationFile - Class to handle permanent storage of configuration
data in human-readable text files.
Copyright (c) 2002-2018 Oliver Kreylos

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

#include <Misc/ConfigurationFile.h>

#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <vector>
#include <Misc/File.h>
#include <Misc/StandardValueCoders.h>

namespace Misc {

/****************************************************************
Methods of class ConfigurationFileBase::MalFormedConfigFileError:
****************************************************************/

ConfigurationFileBase::MalformedConfigFileError::MalformedConfigFileError(const std::string& error,int lineNumber,const std::string& configFileName)
		:std::runtime_error(error+std::string(" in line ")+ValueCoder<int>::encode(lineNumber)+std::string(" of file ")+configFileName)
		{
		}

/************************************************************
Methods of class ConfigurationFileBase::SectionNotFoundError:
************************************************************/

ConfigurationFileBase::SectionNotFoundError::SectionNotFoundError(const std::string& absoluteSectionPath)
	:std::runtime_error(std::string("Configuration file section \"")+absoluteSectionPath+std::string("\" not found"))
	{
	}

/********************************************************
Methods of class ConfigurationFileBase::TagNotFoundError:
********************************************************/

ConfigurationFileBase::TagNotFoundError::TagNotFoundError(const std::string& tagName,const std::string& absoluteSectionPath)
	:std::runtime_error(std::string("Configuration file tag \"")+tagName+std::string("\" not found in section \"")+absoluteSectionPath+std::string("\""))
	{
	}

/***********************************************
Methods of class ConfigurationFileBase::Section:
***********************************************/

ConfigurationFileBase::Section::Section(ConfigurationFileBase::Section* sParent,const std::string& sName)
	:parent(sParent),name(sName),
	 sibling(0),
	 firstSubsection(0),lastSubsection(0),
	 edited(false)
	{
	}

ConfigurationFileBase::Section::~Section(void)
	{
	/* Delete all subsections: */
	while(firstSubsection!=0)
		{
		Section* next=firstSubsection->sibling;
		delete firstSubsection;
		firstSubsection=next;
		}
	}

void ConfigurationFileBase::Section::clear(void)
	{
	/* Remove all subsections: */
	while(firstSubsection!=0)
		{
		Section* succ=firstSubsection->sibling;
		delete firstSubsection;
		firstSubsection=succ;
		}
	lastSubsection=0;
	
	/* Remove all tag/value pairs: */
	values.clear();
	
	/* Mark the section as edited: */
	edited=true;
	}

ConfigurationFileBase::Section* ConfigurationFileBase::Section::addSubsection(const std::string& subsectionName)
	{
	/* Check if the subsection already exists: */
	Section* sPtr;
	for(sPtr=firstSubsection;sPtr!=0;sPtr=sPtr->sibling)
		if(sPtr->name==subsectionName)
			break;
	
	if(sPtr==0)
		{
		/* Add new subsection: */
		Section* newSubsection=new Section(this,subsectionName);
		if(lastSubsection!=0)
			lastSubsection->sibling=newSubsection;
		else
			firstSubsection=newSubsection;
		lastSubsection=newSubsection;
		
		/* Mark the section as edited: */
		edited=true;
		
		return newSubsection;
		}
	else
		return sPtr;
	}

void ConfigurationFileBase::Section::removeSubsection(const std::string& subsectionName)
	{
	/* Find a subsection of the given name: */
	Section* sPred=0;
	Section* sPtr;
	for(sPtr=firstSubsection;sPtr!=0&&sPtr->name!=subsectionName;sPred=sPtr,sPtr=sPtr->sibling)
		;
	if(sPtr!=0)
		{
		/* Remove the subsection: */
		if(sPred!=0)
			sPred->sibling=sPtr->sibling;
		else
			firstSubsection=sPtr->sibling;
		if(sPtr->sibling==0)
			lastSubsection=sPred;
		delete sPtr;
		
		/* Mark the section as edited: */
		edited=true;
		}
	}

void ConfigurationFileBase::Section::addTagValue(const std::string& newTag,const std::string& newValue)
	{
	/* Find the tag name in the section's tag list: */
	std::list<TagValue>::iterator tvIt;
	for(tvIt=values.begin();tvIt!=values.end()&&tvIt->tag!=newTag;++tvIt)
		;
	
	/* Set tag value: */
	if(tvIt==values.end())
		{
		/* Add a new tag/value pair: */
		values.push_back(TagValue(newTag,newValue));
		}
	else
		{
		/* Set new value for existing tag/value pair: */
		tvIt->value=newValue;
		}
	
	/* Mark the section as edited: */
	edited=true;
	}

void ConfigurationFileBase::Section::removeTag(const std::string& tag)
	{
	/* Find the tag name in the section's tag list: */
	std::list<TagValue>::iterator tvIt;
	for(tvIt=values.begin();tvIt!=values.end()&&tvIt->tag!=tag;++tvIt)
		;
	
	/* Check if the tag was found: */
	if(tvIt!=values.end())
		{
		/* Remove tag/value pair: */
		values.erase(tvIt);
		}
	
	/* Mark the section as edited: */
	edited=true;
	}

bool ConfigurationFileBase::Section::isEdited(void) const
	{
	/* Return true if this section has been edited: */
	if(edited)
		return true;
	
	/* Check if any subsections have been edited: */
	for(const Section* sPtr=firstSubsection;sPtr!=0;sPtr=sPtr->sibling)
		if(sPtr->isEdited())
			return true;
	
	/* Nothing has been edited: */
	return false;
	}

void ConfigurationFileBase::Section::clearEditFlag(void)
	{
	/* Clear this section's edit flag: */
	edited=false;
	
	/* Clear edit flags of all subsections: */
	for(Section* sPtr=firstSubsection;sPtr!=0;sPtr=sPtr->sibling)
		sPtr->clearEditFlag();
	}

void ConfigurationFileBase::Section::save(File& file,int sectionLevel)
	{
	/* Generate indentation: */
	char prefix[80];
	for(int i=0;i<sectionLevel;++i)
		prefix[i]='\t';
	prefix[sectionLevel]='\0';
	
	/* Write subsections followed by tag/value pairs: */
	bool didWriteSomething=false;
	
	/* Write all subsections: */
	for(Section* ssPtr=firstSubsection;ssPtr!=0;ssPtr=ssPtr->sibling)
		{
		/* Write separator line: */
		if(didWriteSomething)
			fprintf(file.getFilePtr(),"%s\n",prefix);
		
		/* Write section header: */
		fprintf(file.getFilePtr(),"%ssection %s\n",prefix,ssPtr->name.c_str());
		
		/* Write section contents: */
		ssPtr->save(file,sectionLevel+1);
		
		/* Write section footer: */
		fprintf(file.getFilePtr(),"%sendsection\n",prefix);
		
		didWriteSomething=true;
		}
	
	/* Write tag/value pairs: */
	for(std::list<TagValue>::const_iterator tvIt=values.begin();tvIt!=values.end();++tvIt)
		{
		/* Write separator line: */
		if(didWriteSomething)
			{
			fprintf(file.getFilePtr(),"%s\n",prefix);
			didWriteSomething=false;
			}
		
		/* Write tag/value pair: */
		fprintf(file.getFilePtr(),"%s%s %s\n",prefix,tvIt->tag.c_str(),tvIt->value.c_str());
		}
	
	/* Mark the section as saved: */
	edited=false;
	}

std::string ConfigurationFileBase::Section::getPath(void) const
	{
	if(parent==0)
		return "/";
	else
		{
		/* Accumulate path names while going up the section hierarchy: */
		std::string result="";
		for(const Section* sPtr=this;sPtr->parent!=0;sPtr=sPtr->parent)
			result=std::string("/")+sPtr->name+result;
		return result;
		}
	}

const ConfigurationFileBase::Section* ConfigurationFileBase::Section::getSection(const char* relativePath,const char** pathSuffix) const
	{
	const char* pathSuffixPtr=relativePath;
	const Section* sPtr=this;
	
	/* If the first character in the path is a slash, start searching from the root section: */
	if(pathSuffixPtr[0]=='/')
		{
		while(sPtr->parent!=0)
			sPtr=sPtr->parent;
		++pathSuffixPtr;
		}
	
	/* Process section prefixes of relative path: */
	while(true)
		{
		/* Find next slash in suffix: */
		const char* nextSlashPtr;
		for(nextSlashPtr=pathSuffixPtr;*nextSlashPtr!='\0'&&*nextSlashPtr!='/';++nextSlashPtr)
			;
		if(*nextSlashPtr=='\0'&&pathSuffix!=0)
			{
			/* The slash-free suffix is a tag name; ignore and return it: */
			*pathSuffix=pathSuffixPtr;
			break;
			}
		
		/* Navigate section hierarchy: */
		if(nextSlashPtr-pathSuffixPtr==0)
			{
			/* Ignore double slashes */
			}
		else if(nextSlashPtr-pathSuffixPtr==1&&pathSuffixPtr[0]=='.')
			{
			/* Ignore self-reference */
			}
		else if(nextSlashPtr-pathSuffixPtr==2&&pathSuffixPtr[0]=='.'&&pathSuffixPtr[1]=='.')
			{
			/* Go up in the section hierarchy, if possible: */
			if(sPtr->parent!=0)
				sPtr=sPtr->parent;
			}
		else
			{
			/* Find subsection name in current section: */
			std::string subsectionName(pathSuffixPtr,nextSlashPtr-pathSuffixPtr);
			Section* ssPtr;
			for(ssPtr=sPtr->firstSubsection;ssPtr!=0&&ssPtr->name!=subsectionName;ssPtr=ssPtr->sibling)
				;
			
			/* Go down in the section hierarchy: */
			if(ssPtr==0)
				{
				/* Can't add new section; must throw exception: */
				throw SectionNotFoundError(sPtr->getPath()+std::string("/")+subsectionName);
				}
			else
				sPtr=ssPtr;
			}
		
		if(*nextSlashPtr=='\0')
			break;
		pathSuffixPtr=nextSlashPtr+1;
		}
	
	/* At this point, sPtr points to the correct section, and pathSuffixPtr is empty or slash-free: */
	return sPtr;
	}

ConfigurationFileBase::Section* ConfigurationFileBase::Section::getSection(const char* relativePath,const char** pathSuffix)
	{
	const char* pathSuffixPtr=relativePath;
	Section* sPtr=this;
	
	/* If the first character in the path is a slash, start searching from the root section: */
	if(pathSuffixPtr[0]=='/')
		{
		while(sPtr->parent!=0)
			sPtr=sPtr->parent;
		++pathSuffixPtr;
		}
	
	/* Process section prefixes of relative path: */
	while(true)
		{
		/* Find next slash in suffix: */
		const char* nextSlashPtr;
		for(nextSlashPtr=pathSuffixPtr;*nextSlashPtr!='\0'&&*nextSlashPtr!='/';++nextSlashPtr)
			;
		if(*nextSlashPtr=='\0'&&pathSuffix!=0)
			{
			/* The slash-free suffix is a tag name; ignore and return it: */
			*pathSuffix=pathSuffixPtr;
			break;
			}
		
		/* Navigate section hierarchy: */
		if(nextSlashPtr-pathSuffixPtr==0)
			{
			/* Ignore double slashes */
			}
		else if(nextSlashPtr-pathSuffixPtr==1&&pathSuffixPtr[0]=='.')
			{
			/* Ignore self-reference */
			}
		else if(nextSlashPtr-pathSuffixPtr==2&&pathSuffixPtr[0]=='.'&&pathSuffixPtr[1]=='.')
			{
			/* Go up in the section hierarchy, if possible: */
			if(sPtr->parent!=0)
				sPtr=sPtr->parent;
			}
		else
			{
			/* Go to subsection of given name (create if not already there): */
			sPtr=sPtr->addSubsection(std::string(pathSuffixPtr,nextSlashPtr-pathSuffixPtr));
			}
		
		if(*nextSlashPtr=='\0')
			break;
		pathSuffixPtr=nextSlashPtr+1;
		}
	
	/* At this point, sPtr points to the correct section, and pathSuffixPtr is empty or slash-free: */
	return sPtr;
	}

bool ConfigurationFileBase::Section::hasTag(const char* relativeTagPath) const
	{
	/* Go to the section containing the given tag: */
	const char* tagName=0;
	const Section* sPtr=getSection(relativeTagPath,&tagName);
	
	/* Find the tag name in the section's tag list: */
	std::list<TagValue>::const_iterator tvIt;
	for(tvIt=sPtr->values.begin();tvIt!=sPtr->values.end()&&tvIt->tag!=tagName;++tvIt)
		;
	
	return tvIt!=sPtr->values.end();
	}

const std::string* ConfigurationFileBase::Section::findTagValue(const char* relativeTagPath) const
	{
	/* Go to the section containing the given tag: */
	const char* tagName=0;
	const Section* sPtr=getSection(relativeTagPath,&tagName);
	
	/* Find the tag name in the section's tag list: */
	std::list<TagValue>::const_iterator tvIt;
	for(tvIt=sPtr->values.begin();tvIt!=sPtr->values.end()&&tvIt->tag!=tagName;++tvIt)
		;
	
	/* Return tag value or null pointer: */
	return tvIt!=sPtr->values.end()?&(tvIt->value):0;
	}

const std::string& ConfigurationFileBase::Section::retrieveTagValue(const char* relativeTagPath) const
	{
	/* Go to the section containing the given tag: */
	const char* tagName=0;
	const Section* sPtr=getSection(relativeTagPath,&tagName);
	
	/* Find the tag name in the section's tag list: */
	std::list<TagValue>::const_iterator tvIt;
	for(tvIt=sPtr->values.begin();tvIt!=sPtr->values.end()&&tvIt->tag!=tagName;++tvIt)
		;
	
	/* Return tag value: */
	if(tvIt==sPtr->values.end())
		throw TagNotFoundError(tagName,sPtr->getPath());
	return tvIt->value;
	}

std::string ConfigurationFileBase::Section::retrieveTagValue(const char* relativeTagPath,const std::string& defaultValue) const
	{
	const char* tagName=0;
	const Section* sPtr;
	try
		{
		/* Go to the section containing the given tag: */
		sPtr=getSection(relativeTagPath,&tagName);
		}
	catch(const SectionNotFoundError& error)
		{
		/* If the section does not exist, bail out and return the default value: */
		return defaultValue;
		}
	
	/* Find the tag name in the section's tag list: */
	std::list<TagValue>::const_iterator tvIt;
	for(tvIt=sPtr->values.begin();tvIt!=sPtr->values.end()&&tvIt->tag!=tagName;++tvIt)
		;

	/* Return tag value: */
	if(tvIt==sPtr->values.end())
		throw TagNotFoundError(tagName,sPtr->getPath());
	return tvIt->value;
	}

const std::string& ConfigurationFileBase::Section::retrieveTagValue(const char* relativeTagPath,const std::string& defaultValue)
	{
	/* Go to the section containing the given tag: */
	const char* tagName=0;
	Section* sPtr=getSection(relativeTagPath,&tagName);
	
	/* Find the tag name in the section's tag list: */
	std::list<TagValue>::const_iterator tvIt;
	for(tvIt=sPtr->values.begin();tvIt!=sPtr->values.end()&&tvIt->tag!=tagName;++tvIt)
		;
	
	/* Return tag value: */
	if(tvIt==sPtr->values.end())
		{
		/* Add a new tag/value pair: */
		sPtr->values.push_back(TagValue(tagName,defaultValue));
		
		/* Mark section as edited: */
		sPtr->edited=true;
		
		return defaultValue;
		}
	else
		return tvIt->value;
	}

void ConfigurationFileBase::Section::storeTagValue(const char* relativeTagPath,const std::string& newValue)
	{
	/* Go to the section containing the given tag: */
	const char* tagName=0;
	Section* sPtr=getSection(relativeTagPath,&tagName);
	
	/* Add the tag/value pair: */
	sPtr->addTagValue(tagName,newValue);
	}

/**************************************
Methods of class ConfigurationFileBase:
**************************************/

ConfigurationFileBase::ConfigurationFileBase(void)
	:rootSection(new Section(0,std::string("")))
	{
	}

ConfigurationFileBase::ConfigurationFileBase(const char* sFileName)
	:rootSection(0)
	{
	/* Load the configuration file: */
	load(sFileName);
	}

ConfigurationFileBase::~ConfigurationFileBase(void)
	{
	delete rootSection;
	}

void ConfigurationFileBase::load(const char* newFileName)
	{
	/* Delete current configuration file contents: */
	delete rootSection;
	
	/* Create root section: */
	rootSection=new Section(0,std::string(""));
	
	/* Store the file name: */
	fileName=newFileName;
	
	/* Merge contents of given configuration file: */
	merge(newFileName);
	
	/* Reset edit flag: */
	rootSection->clearEditFlag();
	}

void ConfigurationFileBase::merge(const char* mergeFileName)
	{
	/* Try opening configuration file: */
	File file(mergeFileName,"rt");
	
	/* Read configuration file contents: */
	Section* sectionPtr=rootSection;
	int lineNumber=0;
	while(!file.eof())
		{
		/* Concatenate lines from configuration file: */
		std::string line="";
		char lineBuffer[1024];
		bool firstLine=true;
		while(true)
			{
			/* Read line from file: */
			if(file.gets(lineBuffer,sizeof(lineBuffer))==0)
				break;
			++lineNumber;
			
			/* Skip initial whitespace: */
			char* lineStartPtr;
			for(lineStartPtr=lineBuffer;*lineStartPtr!='\0'&&isspace(*lineStartPtr);++lineStartPtr)
				;
			
			/* Find end of line: */
			char* lineEndPtr;
			for(lineEndPtr=lineStartPtr;*lineEndPtr!='\0';++lineEndPtr)
				;
			
			/* Check if line was read completely: */
			if(lineEndPtr[-1]!='\n')
				throw MalformedConfigFileError("Line too long",lineNumber,fileName);
			
			if(lineEndPtr-2>=lineBuffer&&lineEndPtr[-2]=='\\')
				{
				/* Remove the continuation characters "\\\n" from the buffer: */
				lineEndPtr[-2]='\0';
				
				/* Concatenate the current buffer to the line and continue reading: */
				if(firstLine||*lineStartPtr!='#')
					line+=lineStartPtr;
				}
			else
				{
				/* Remove the line end character from the buffer: */
				lineEndPtr[-1]='\0';
				
				/* Concatenate the current buffer to the line and stop reading: */
				if(firstLine||*lineStartPtr!='#')
					line+=lineStartPtr;
				break;
				}
			
			firstLine=false;
			}
		
		/* Get pointers to beginning and end of line: */
		const char* linePtr=line.data();
		const char* lineEndPtr=linePtr+line.size();
		
		/* Check if the line contains a comment: */
		for(const char* lPtr=linePtr;lPtr!=lineEndPtr;++lPtr)
			if(*lPtr=='#')
				{
				lineEndPtr=lPtr;
				break;
				}
		
		/* Remove whitespace from the end of the line: */
		while(lineEndPtr!=linePtr&&isspace(lineEndPtr[-1]))
			--lineEndPtr;
		
		/* Check for empty lines: */
		if(linePtr==lineEndPtr)
			continue;
		
		/* Extract first string from line: */
		const char* decodeEnd;
		std::string token=ValueCoder<std::string>::decode(linePtr,lineEndPtr,&decodeEnd);
		linePtr=decodeEnd;
		while(linePtr!=lineEndPtr&&isspace(*linePtr))
			++linePtr;
		
		if(strcasecmp(token.c_str(),"section")==0)
			{
			/* Check if the section name starts with a double quote for backwards compatibility: */
			std::string sectionName;
			if(linePtr!=lineEndPtr&&*linePtr=='\"')
				{
				/* Parse the section name as a string: */
				sectionName=ValueCoder<std::string>::decode(linePtr,lineEndPtr,&decodeEnd);
				}
			else
				{
				/* Read everything after the "section" token as the section name, including whitespace and special characters: */
				sectionName=std::string(linePtr,lineEndPtr);
				}
			
			/* Add a new subsection to the current section and make it the current section: */
			if(sectionName.empty())
				throw MalformedConfigFileError("Missing section name after section command",lineNumber,fileName);
			sectionPtr=sectionPtr->addSubsection(sectionName);
			}
		else if(strcasecmp(token.c_str(),"endsection")==0)
			{
			/* End the current section: */
			if(sectionPtr->parent!=0)
				sectionPtr=sectionPtr->parent;
			else
				throw MalformedConfigFileError("Extra endsection command",lineNumber,fileName);
			}
		else if(linePtr!=lineEndPtr)
			{
			/* Check for the special "+=" operator: */
			if(*linePtr=='+'&&linePtr+1!=lineEndPtr&&linePtr[1]=='=')
				{
				/* Skip the operator and whitespace and get the new tag value: */
				for(linePtr+=2;linePtr!=lineEndPtr&&isspace(*linePtr);++linePtr)
					;
				if(linePtr!=lineEndPtr)
					{
					/* Get the current tag value, defaulting to an empty list if the tag does not exist yet: */
					std::string currentValue=sectionPtr->retrieveTagValue(token.c_str(),"()");
					
					/* Check that the current tag ends with a closing parenthesis, and the new tag value starts with an opening parenthesis: */
					if(*linePtr=='('&&*(currentValue.end()-1)==')')
						{
						/* Concatenate the current and new tag values: */
						currentValue.erase(currentValue.end()-1);
						
						/* Insert a list item separator if the current value is not the empty list: */
						if(*(currentValue.end()-1)!='(')
							currentValue.append(", ");
						
						currentValue.append(std::string(linePtr+1,lineEndPtr));
						
						/* Store the concatenated tag values: */
						sectionPtr->addTagValue(token,currentValue);
						}
					else
						throw MalformedConfigFileError("+= operator used on non-list",lineNumber,fileName);
					}
				}
			else
				{
				/* Add a tag/value pair to the current section: */
				sectionPtr->addTagValue(token,std::string(linePtr,lineEndPtr));
				}
			}
		else
			{
			/* Remove the tag from the current section: */
			sectionPtr->removeTag(token);
			}
		}
	}

void ConfigurationFileBase::mergeCommandline(int& argc,char**& argv)
	{
	/* Process all command line arguments: */
	for(int i=1;i<argc;++i)
		{
		/* Check for a tag: */
		if(argv[i][0]=='-')
			{
			if(i<argc-1)
				{
				/* Add the tag/value pair to the root section: */
				rootSection->storeTagValue(argv[i]+1,argv[i+1]);
				
				/* Remove the tag and value from the command line: */
				argc-=2;
				for(int j=i;j<argc;++j)
					argv[j]=argv[j+2];
				i-=2;
				}
			else
				{
				/* Remove the solo tag from the command line: */
				--argc;
				--i;
				}
			}
		}
	}

void ConfigurationFileBase::saveAs(const char* newFileName)
	{
	/* Store the new file name: */
	fileName=newFileName;
	
	/* Save the root section: */
	File file(fileName.c_str(),"wt");
	rootSection->save(file,0);
	}

namespace {

/****************
Helper functions:
****************/

char processEscape(int c)
	{
	switch(c)
		{
		case 'a':
			return '\a';
		
		case 'b':
			return '\b';
		
		case 'f':
			return '\f';
		
		case 'n':
			return '\n';
		
		case 'r':
			return '\r';
		
		case 't':
			return '\t';
		
		case 'v':
			return '\v';
		
		default:
			return char(c);
		}
	}

/**************
Helper classes:
**************/

struct SectionMatch // Structure defining the match of a section with the tag path
	{
	/* Elements: */
	public:
	std::string name; // The section name
	const char* matchedPrefix; // Pointer to the first component of the tag path that is not matched by this section, or NULL
	const char* nextSlash; // Pointer to the next slash or end-of-string in the tag path
	
	/* Constructors and destructors: */
	SectionMatch(const char* sNameBegin,const char* sNameEnd,const char* sMatchedPrefix =0,const char* sNextSlash =0)
		:name(sNameBegin,sNameEnd),
		 matchedPrefix(sMatchedPrefix),nextSlash(sNextSlash)
		{
		}
	};

}

void ConfigurationFileBase::patchFile(const char* fileName,const char* tagPath,const char* newValue)
	{
	/* Open a temporary output file: */
	size_t fnLen=strlen(fileName);
	char* tempFileName=new char[fnLen+6+1];
	memcpy(tempFileName,fileName,fnLen);
	for(int i=0;i<6;++i)
		tempFileName[fnLen+i]='X';
	tempFileName[fnLen+6]='\0';
	int tempFd=mkstemp(tempFileName);
	if(tempFd<0)
		{
		int error=errno;
		throwStdErr("Misc::ConfigurationFile::patchFile: Unable to patch file %s due to error %d (%s)",fileName,error,strerror(error));
		}
	
	try
		{
		/* Put a File wrapper around the temporary file: */
		File tempFile(tempFd,"w+");
		
		/* Try opening the given configuration file: */
		File file(fileName,"rt");
		
		/* Parse the given configuration file: */
		enum State // States of the file parser automaton
			{
			LINE, // At beginning of line, reading whitespace
			TAG,QUOTEDTAG, // Parsing a naked or quoted tag
			SECTIONWS, // Reading whitespace between the "section" keyword and the section name
			SECTION,QUOTEDSECTION, // Reading a naked or quoted section name
			VALUEWS, // Reading whitespace between a tag and its value
			VALUEWSPLUS, // Read a "+" between a tag and its value
			VALUEWSPLUSEQUAL, // Read a "+=" between a tag and its value
			VALUE, // Parsing a value
			VALUESKIPWS, // Skipping whitespace inside a value due to a line break
			SKIPVALUE, // Skipping the value of a matched tag
			COMMENT, // Skipping a comment
			SKIPLINE // Skipping until the end of the current line
			} state=LINE;
		unsigned int lineNumber=1;
		std::vector<SectionMatch> sections; // Stack of section names
		const char* matchedPrefix=tagPath; // Pointer to the first character of the tag path not yet matched to the current section path
		const char* nextSlash; // Pointer to the next slash (or end of string) in the tag path
		for(nextSlash=matchedPrefix;*nextSlash!='/'&&*nextSlash!='\0';++nextSlash)
			;
		sections.push_back(SectionMatch(tagPath,tagPath,matchedPrefix,nextSlash)); // A root section as sentinel
		char whitespace[1024]; // Whitespace at the beginning of the current line
		char* wsPtr=whitespace;
		char tag[1024]; // The currently-parsed tag
		char* tPtr=tag;
		char section[1024]; // The currently-parsed section name
		char* sPtr=section;
		std::string value; // The currently-parsed value
		// bool valueContinuation=false; // Flag whether the current value is a list continuation
		bool escaped=false;
		int quote=-1;
		bool copyChars=true;
		int c;
		while((c=file.getc())>=0)
			{
			switch(state)
				{
				case LINE:
					if(c=='\n')
						{
						++lineNumber;
						wsPtr=whitespace;
						}
					else if(c=='#')
						state=COMMENT;
					else if(c=='"'||c=='\'')
						{
						quote=c;
						state=QUOTEDTAG;
						tPtr=tag;
						}
					else if(isspace(c))
						*(wsPtr++)=char(c);
					else
						{
						state=TAG;
						tPtr=tag;
						if(c=='\\')
							escaped=true;
						else
							*(tPtr++)=char(c);
						}
					break;
				
				case TAG:
					if(escaped)
						{
						if(c=='\n')
							++lineNumber;
						else
							*(tPtr++)=processEscape(c);
						escaped=false;
						}
					else if(c=='\\')
						escaped=true;
					else if(c=='\n'||isspace(c))
						{
						/* Tag is finished; process it: */
						*tPtr='\0';
						if(strcasecmp(tag,"section")==0)
							{
							if(c=='\n')
								throw MalformedConfigFileError("Missing section name",lineNumber,fileName);
							else
								state=SECTIONWS;
							}
						else if(strcasecmp(tag,"endsection")==0)
							{
							/* Pop the current section off the stack: */
							if(sections.size()<=1)
								throw MalformedConfigFileError("Extra endsection command",lineNumber,fileName);
							sections.pop_back();
							if(sections.back().matchedPrefix!=0)
								{
								matchedPrefix=sections.back().matchedPrefix;
								nextSlash=sections.back().nextSlash;
								}
							
							if(c=='\n')
								{
								/* Parse the next line: */
								++lineNumber;
								state=LINE;
								wsPtr=whitespace;
								}
							else
								state=SKIPLINE;
							}
						else
							{
							/* Check if the current section path matches the tag path: */
							if(sections.back().matchedPrefix!=0&&*nextSlash=='\0')
								{
								/* Match the tag against the tag path's suffix: */
								char* cPtr1;
								const char* cPtr2;
								for(cPtr1=tag,cPtr2=matchedPrefix;cPtr1<tPtr&&cPtr2<nextSlash&&*cPtr1==*cPtr2;++cPtr1,++cPtr2)
									;
								if(cPtr1==tPtr&&cPtr2==nextSlash)
									{
									/* Output the replacement value to the temporary file: */
									fputc(' ',tempFile.getFilePtr());
									for(const char* vPtr=newValue;*vPtr!='\0';++vPtr)
										{
										if(*vPtr=='\n')
											{
											/* Start a new line and indent it to the tag's indentation: */
											fputc('\n',tempFile.getFilePtr());
											for(char* iPtr=whitespace;iPtr!=wsPtr;++iPtr)
												fputc(*iPtr,tempFile.getFilePtr());
											
											}
										else
											fputc(*vPtr,tempFile.getFilePtr());
										}
									
									/* Stop copying characters and skip the tag's value: */
									copyChars=false;
									state=SKIPVALUE;
									}
								else
									state=VALUEWS;
								}
							else
								state=VALUEWS;
							}
						
						if(c=='\n')
							{
							/* Parse the next line: */
							++lineNumber;
							copyChars=true;
							state=LINE;
							wsPtr=whitespace;
							}
						}
					else
						*(tPtr++)=char(c);
					break;
				
				case QUOTEDTAG:
					if(escaped)
						{
						if(c=='\n')
							++lineNumber;
						else
							*(tPtr++)=processEscape(c);
						escaped=false;
						}
					else if(c=='\\')
						escaped=true;
					else if(c!=quote)
						*(tPtr++)=char(c);
					else
						{
						/* Tag is finished; process it: */
						*tPtr='\0';
						if(strcasecmp(tag,"section")==0)
							state=SECTIONWS;
						else if(strcasecmp(tag,"endsection")==0)
							{
							/* Pop the current section off the stack: */
							if(sections.size()<=1)
								throw MalformedConfigFileError("Extra endsection command",lineNumber,fileName);
							sections.pop_back();
							if(sections.back().matchedPrefix!=0)
								{
								matchedPrefix=sections.back().matchedPrefix;
								nextSlash=sections.back().nextSlash;
								}
							
							/* Skip the rest of the line: */
							state=SKIPLINE;
							}
						else
							{
							/* Check if the current section path matches the tag path: */
							if(sections.back().matchedPrefix!=0&&*nextSlash=='\0')
								{
								/* Match the tag against the tag path's suffix: */
								char* cPtr1;
								const char* cPtr2;
								for(cPtr1=tag,cPtr2=matchedPrefix;cPtr1<tPtr&&cPtr2<nextSlash&&*cPtr1==*cPtr2;++cPtr1,++cPtr2)
									;
								if(cPtr1==tPtr&&cPtr2==nextSlash)
									{
									/* Output the replacement value to the temporary file: */
									fputc(quote,tempFile.getFilePtr());
									fputc(' ',tempFile.getFilePtr());
									for(const char* vPtr=newValue;*vPtr!='\0';++vPtr)
										{
										if(*vPtr=='\n')
											{
											/* Start a new line and indent it to the tag's indentation: */
											fputc('\n',tempFile.getFilePtr());
											for(char* iPtr=whitespace;iPtr!=wsPtr;++iPtr)
												fputc(*iPtr,tempFile.getFilePtr());
											
											}
										else
											fputc(*vPtr,tempFile.getFilePtr());
										}
									
									/* Stop copying characters and skip the tag's value: */
									copyChars=false;
									state=SKIPVALUE;
									}
								else
									state=VALUEWS;
								}
							else
								state=VALUEWS;
							}
						}
					break;
				
				case SECTIONWS:
					if(c=='\n'||c=='#')
						throw MalformedConfigFileError("Missing section name",lineNumber,fileName);
					else if(c=='"'||c=='\'')
						{
						quote=c;
						state=QUOTEDSECTION;
						sPtr=section;
						}
					else if(!isspace(c))
						{
						state=SECTION;
						sPtr=section;
						if(c=='\\')
							escaped=true;
						else
							*(sPtr++)=char(c);
						}
					break;
				
				case SECTION:
					if(escaped)
						{
						if(c=='\n')
							++lineNumber;
						else
							*(sPtr++)=processEscape(c);
						escaped=false;
						}
					else if(c=='\\')
						escaped=true;
					else if(c=='\n'||c=='#')
						{
						/* Section name is finished, remove trailing whitespace and process it: */
						while(sPtr>section&&isspace(sPtr[-1]))
							--sPtr;
						SectionMatch sm(section,sPtr);
						
						/* Check if the current section path matches a prefix of the given tag path: */
						if(sections.back().matchedPrefix!=0&&*nextSlash!='\0')
							{
							/* Match this section name against the unmatched suffix of the tag path: */
							char* cPtr1;
							const char* cPtr2;
							for(cPtr1=section,cPtr2=matchedPrefix;cPtr1<sPtr&&cPtr2<nextSlash&&*cPtr1==*cPtr2;++cPtr1,++cPtr2)
								;
							if(cPtr1==sPtr&&cPtr2==nextSlash)
								{
								matchedPrefix=nextSlash+1;
								for(nextSlash=matchedPrefix;*nextSlash!='/'&&*nextSlash!='\0';++nextSlash)
									;
								sm.matchedPrefix=matchedPrefix;
								sm.nextSlash=nextSlash;
								}
							}
						
						/* Push the section onto the stack: */
						sections.push_back(sm);
						
						if(c=='#')
							state=COMMENT;
						else
							{
							/* Parse the next line: */
							++lineNumber;
							state=LINE;
							wsPtr=whitespace;
							}
						}
					else
						*(sPtr++)=char(c);
					break;
				
				case QUOTEDSECTION:
					if(escaped)
						{
						if(c=='\n')
							++lineNumber;
						else
							*(sPtr++)=processEscape(c);
						escaped=false;
						}
					else if(c=='\\')
						escaped=true;
					else if(c==quote)
						{
						/* Section name is finished, process it: */
						SectionMatch sm(section,sPtr);
						
						/* Check if the current section path matches a prefix of the given tag path: */
						if(sections.back().matchedPrefix!=0&&*nextSlash!='\0')
							{
							/* Match this section name against the unmatched suffix of the tag path: */
							char* cPtr1;
							const char* cPtr2;
							for(cPtr1=section,cPtr2=matchedPrefix;cPtr1<sPtr&&cPtr2<nextSlash&&*cPtr1==*cPtr2;++cPtr1,++cPtr2)
								;
							if(cPtr1==sPtr&&cPtr2==nextSlash)
								{
								matchedPrefix=nextSlash+1;
								for(nextSlash=matchedPrefix;*nextSlash!='/'&&*nextSlash!='\0';++nextSlash)
									;
								sm.matchedPrefix=matchedPrefix;
								sm.nextSlash=nextSlash;
								}
							}
						
						/* Push the section onto the stack: */
						sections.push_back(sm);
						
						/* Skip the rest of the line: */
						state=SKIPLINE;
						}
					else
						*(sPtr++)=char(c);
					break;
				
				case VALUEWS:
					if(c=='\n'||c=='#')
						{
						/* Value is empty; do nothing */
						
						if(c=='#')
							state=COMMENT;
						else
							{
							/* Parse next line: */
							++lineNumber;
							state=LINE;
							wsPtr=whitespace;
							}
						}
					else if(c=='+')
						state=VALUEWSPLUS;
					else if(!isspace(c))
						{
						state=VALUE;
						value.clear();
						if(c=='\\')
							escaped=true;
						else
							value.push_back(char(c));
						// valueContinuation=false;
						}
					break;
				
				case VALUEWSPLUS:
					if(c=='=')
						state=VALUEWSPLUSEQUAL;
					else
						throw MalformedConfigFileError("Malformed += list continuation",lineNumber,fileName);
					break;
				
				case VALUEWSPLUSEQUAL:
					if(c=='(')
						{
						state=VALUE;
						value.clear();
						// valueContinuation=true;
						}
					else if(c=='\n'||!isspace(c))
						throw MalformedConfigFileError("Malformed += list continuation",lineNumber,fileName);
					break;
				
				case VALUE:
					if(escaped)
						{
						if(c=='\n')
							{
							++lineNumber;
							state=VALUESKIPWS;
							}
						else
							{
							/* Escape sequences other than line breaks are copied verbatim to be parsed by value coders: */
							value.push_back('\\');
							value.push_back(char(c));
							}
						escaped=false;
						}
					else if(c=='\\')
						escaped=true;
					else if(c=='"'||c=='\'')
						{
						if(quote==c)
							{
							/* End a quote: */
							quote=-1;
							}
						else if(quote==-1)
							{
							/* Start a quote: */
							quote=c;
							}
						value.push_back(c);
						}
					else if(c=='\n'||(c=='#'&&quote==-1))
						{
						/* Value is finished */
						
						if(c=='#')
							state=COMMENT;
						else
							{
							/* Parse next line: */
							++lineNumber;
							state=LINE;
							wsPtr=whitespace;
							}
						}
					else
						value.push_back(c);
					break;
				
				case VALUESKIPWS:
					if(c=='\\')
						{
						state=VALUE;
						escaped=true;
						}
					else if(c=='"'||c=='\'')
						{
						if(quote==c)
							{
							/* End a quote: */
							quote=-1;
							}
						else if(quote==-1)
							{
							/* Start a quote: */
							quote=c;
							}
						state=VALUE;
						value.push_back(c);
						}
					else if(c=='\n'||(c=='#'&&quote==-1))
						{
						/* Value is finished */
						
						if(c=='#')
							state=COMMENT;
						else
							{
							/* Parse next line: */
							++lineNumber;
							state=LINE;
							wsPtr=whitespace;
							}
						}
					else if(!isspace(c))
						{
						state=VALUE;
						value.push_back(c);
						}
					break;
				
				case SKIPVALUE:
					if(escaped)
						{
						if(c=='\n')
							++lineNumber;
						escaped=false;
						}
					else if(c=='\\')
						escaped=true;
					else if(c=='"'||c=='\'')
						{
						if(quote==c)
							{
							/* End a quote: */
							quote=-1;
							}
						else if(quote==-1)
							{
							/* Start a quote: */
							quote=c;
							}
						}
					else if(c=='\n'||(c=='#'&&quote==-1))
						{
						/* Value is skipped: */
						copyChars=true;
						
						if(c=='#')
							state=COMMENT;
						else
							{
							/* Parse the next line: */
							++lineNumber;
							state=LINE;
							wsPtr=whitespace;
							}
						}
					break;
				
				case COMMENT:
					if(c=='\n')
						{
						/* Parse the next line: */
						++lineNumber;
						state=LINE;
						wsPtr=whitespace;
						}
					break;
				
				case SKIPLINE:
					if(c=='\n')
						{
						/* Parse the next line: */
						++lineNumber;
						state=LINE;
						wsPtr=whitespace;
						}
					else if(c=='#')
						state=COMMENT;
					else if(!isspace(c))
						throw MalformedConfigFileError("Dangling bits at line end",lineNumber,fileName);
					break;
				}
			
			if(copyChars)
				fputc(c,tempFile.getFilePtr());
			}
		}
	catch(...)
		{
		/* Delete the temporary file: */
		unlink(tempFileName);
		
		/* Re-throw the exception: */
		throw;
		}
	
	/* Atomically replace the original file with the temporary file: */
	if(rename(tempFileName,fileName)!=0)
		{
		/* Delete the temporary file and throw an exception: */
		int error=errno;
		unlink(tempFileName);
		throwStdErr("Misc::ConfigurationFile::patchFile: Unable to patch file %s due to error %d (%s)",fileName,error,strerror(error));
		}
	}

/*****************************************
Methods of class ConfigurationFileSection:
*****************************************/

std::string ConfigurationFileSection::getPath(void) const
	{
	return baseSection->getPath();
	}

void ConfigurationFileSection::setSection(const char* relativePath)
	{
	baseSection=baseSection->getSection(relativePath);
	}

ConfigurationFileSection ConfigurationFileSection::getSection(const char* relativePath) const
	{
	return ConfigurationFileSection(baseSection->getSection(relativePath));
	}

void ConfigurationFileSection::clear(void)
	{
	baseSection->clear();
	}

void ConfigurationFileSection::removeSubsection(const std::string& subsectionName)
	{
	baseSection->removeSubsection(subsectionName);
	}

void ConfigurationFileSection::removeTag(const std::string& tag)
	{
	baseSection->removeTag(tag);
	}

/**********************************
Methods of class ConfigurationFile:
**********************************/

void ConfigurationFile::load(const char* newFileName)
	{
	/* Call base class method: */
	ConfigurationFileBase::load(newFileName);
	
	/* Reset the current section pointer to the root section: */
	baseSection=rootSection;
	}

std::string ConfigurationFile::getCurrentPath(void) const
	{
	return baseSection->getPath();
	}

void ConfigurationFile::setCurrentSection(const char* relativePath)
	{
	baseSection=baseSection->getSection(relativePath);
	}

ConfigurationFileSection ConfigurationFile::getCurrentSection(void) const
	{
	return ConfigurationFileSection(baseSection);
	}

ConfigurationFileSection ConfigurationFile::getSection(const char* relativePath)
	{
	return ConfigurationFileSection(baseSection->getSection(relativePath));
	}

void ConfigurationFile::list(void) const
	{
	/* List all subsections of current section: */
	for(Section* sPtr=baseSection->firstSubsection;sPtr!=0;sPtr=sPtr->sibling)
		printf("%s/\n",sPtr->name.c_str());
	
	/* List all tags in current section: */
	for(std::list<Section::TagValue>::const_iterator tvIt=baseSection->values.begin();tvIt!=baseSection->values.end();++tvIt)
		printf("%s\n",tvIt->tag.c_str());
	}

}
