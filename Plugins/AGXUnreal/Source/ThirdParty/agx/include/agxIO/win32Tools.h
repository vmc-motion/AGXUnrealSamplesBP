/*
Copyright 2007-2024. Algoryx Simulation AB.

All AGX source code, intellectual property, documentation, sample code,
tutorials, scene files and technical white papers, are copyrighted, proprietary
and confidential material of Algoryx Simulation AB. You may not download, read,
store, distribute, publish, copy or otherwise disseminate, use or expose this
material unless having a written signed agreement with Algoryx Simulation AB, or having been
advised so by Algoryx Simulation AB for a time limited evaluation, or having purchased a
valid commercial license from Algoryx Simulation AB.

Algoryx Simulation AB disclaims all responsibilities for loss or damage caused
from using this software, unless otherwise stated in written agreements with
Algoryx Simulation AB.
*/

#ifndef AGXIO_WIN32_TOOLS_H
#define AGXIO_WIN32_TOOLS_H

#include <agx/config.h>


#ifdef STANDALONE_RUNTIME
#include <agx/agxCore_export.h>

DOXYGEN_START_INTERNAL_BLOCK()
namespace agx
{
  typedef std::string String;
}
DOXYGEN_END_INTERNAL_BLOCK()
#else
#include <agx/String.h>
#endif
#if defined(_MSC_VER)
#include <agx/Windows.h>
#endif



namespace agxIO
{
  /**
  Get a string value from the registry written by the Installer
  \param key - Name of the key
  \param value - At success, the value will contain the string content of the key.
  \return true if the key exist and is of type string and we have read access to it.
  */
  AGXCORE_EXPORT bool getRegisterValue( const char *key, agx::String& value );

#if defined(_MSC_VER)
  /**
  Get a string value from the registry given by the path and key
  \param parentKey - parent key (e.g., HKEY_CURRENT_USER or HKEY_LOCAL_MACHINE)
  \param path - Path of registry key
  \param key - Name of the key
  \param value - At success, the value will contain the string content of the key.
  \return true if the key exist and is of type string and we have read access to it.
  */
  AGXCORE_EXPORT bool getRegisterValue(HKEY parentKey, const char* path, const char* key, agx::String& value);

  /**
  Set a string value to the given registry path and key
  \param parentKey - parent key (e.g., HKEY_CURRENT_USER or HKEY_LOCAL_MACHINE)
  \param path - Path of registry key to create
  \param key - The registry key
  \param value - The registry key value
  \return true if the key was successfully saved, else false.
  */
  AGXCORE_EXPORT bool setRegisterValue(HKEY parentKey, const char* path, const char* key, agx::String& value);

  /**
  Delete a registry key with given registry path
  \param parentKey - parent key (e.g., HKEY_CURRENT_USER or HKEY_LOCAL_MACHINE)
  \param path - Path of registry key to create
  \param key - The registry key  
  \return true if the key was successfully deleted, else false.
  */
  AGXCORE_EXPORT bool deleteRegistryKey(HKEY parentKey, const char* path, const char* key);

  /**
  Delete a registry tree at the given path
  \param parentKey - parent key (e.g., HKEY_CURRENT_USER or HKEY_LOCAL_MACHINE)
  \param path - Path of registry key to create
  \param key - The registry key
  \return true if the key was successfully deleted, else false.
  */
  AGXCORE_EXPORT bool deleteRegistryTree(HKEY parentKey, const char* path);
#endif
}
#endif

