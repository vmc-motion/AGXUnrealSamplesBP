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

#ifndef AGX_LOCALHASHTABLE_H
#define AGX_LOCALHASHTABLE_H

#include <agx/HashTable.h>
#include <agx/Vector.h>

// Disabled until bug in HashSet::rebuild is fixed, old buffer may be invalidated if thread scratch pad is reallocated
#if 0
namespace agxData
{
  /**
  Local scope hash table. Fast allocation/deallocation in local thread stack.
  */
  template <typename KeyT, typename DataT, typename HashT = agx::DefaultHashFunctions>
  class LocalHashTable : public agx::HashTable<KeyT, DataT, HashT>
  {
  public:
    explicit LocalHashTable();
  };


  /* Implementation */
  template <typename KeyT, typename DataT, typename HashT>
  AGX_FORCE_INLINE LocalHashTable<KeyT, DataT, HashT>::LocalHashTable() : agx::HashTable<KeyT, DataT, HashT, agx::Thread::LocalAllocator>() {}

}

// AGX_TEMPLATED_TYPE_BINDING(agxData::LocalHashTable, "LocalHashTable")
#endif

#endif /* AGX_LOCALHASHTABLE_H */
