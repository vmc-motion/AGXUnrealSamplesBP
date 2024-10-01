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

#ifndef AGX_HASHVECTOR_H
#define AGX_HASHVECTOR_H


#include <agx/Vector.h>
#include <agx/HashTable.h>

namespace agx
{

  /**
  This class is a combined container which has the find complexity of a HashTable,
  deterministic iteration of a Vector (using begin(), end()).
  The cost is storage requirement. Data is stored in a vector<pair<KeyT, DataT> >, which contains the Hash key and the data.
  A hash table in parallel store the index to the vector and the corresponding key.

  Beware of accessing the iterator of the vector, and change the Key value (pair.first). This will effectively invalidate the
  data structure. Unfortunately, there is no easy way protecting this unless only a const_iterator is used for accessing the vector, which
  would reduce the practical functionality of the container.

  The vector contains a std::pair<KeyT, DataT>, so the iterator is a pointer to such an object.
  */
  template <typename KeyT, typename DataT=KeyT, typename HashT = agx::HashFn<KeyT> >
  class HashVector
  {
  public:

    typedef DataT value_type;
    typedef HashVector<KeyT, DataT, HashT> ContainerType;

    typedef Vector<std::pair<KeyT, DataT> > VectorType;
    typedef HashTable<KeyT, size_t, HashT> HashType;

  private:

    VectorType m_vector;
    HashType m_hash;

  public:

    enum ClearPolicy
    {
      SHRINK_BUFFER=VectorType::SHRINK_BUFFER,           // Buffer is deallocated and replaced by an empty buffer
      SHRINK_BUFFER_AVERAGED=VectorType::SHRINK_BUFFER_AVERAGED,  // Buffer is shrunk if a smoothing average (which is updated each clear call) goes below a threshold
      MAINTAIN_BUFFER=VectorType::MAINTAIN_BUFFER          // Buffer is maintained
    };

    typedef typename VectorType::iterator iterator;
    typedef typename VectorType::const_iterator const_iterator;


    /** Constructor */
    HashVector()
    {}

    /** Copy constructor */
    HashVector(const ContainerType& other) : m_vector( other.m_vector ), m_hash( other.m_hash )
    {
      this->clear(SHRINK_BUFFER);
      *this = other;
    }

    /** Assignment operator */
    HashVector<KeyT, DataT, HashT>& operator= (const ContainerType& other)
    {
      if ( this == &other ) {
        return *this;
      }

      m_vector = other.m_vector;
      m_hash = other.m_hash;
      return *this;
    }


    /** Destructor */
    ~HashVector()
    {
      // don't change buffer, will be dealloc'ed soon anyway
      clear(MAINTAIN_BUFFER);
    }

    const VectorType& vector() const
    {
      return m_vector;
    }

    const HashType& table() const
    {
      return m_hash;
    }

    /**
    Return the number of inserted elements.
    */
    size_t size() const
    {
      agxAssert( checkSize() );

      return m_vector.size();
    }

    /**
    Return true if the container is empty
    */
    bool empty() const
    {
      return m_vector.empty();
    }

    /**
    Insert a key,data pair in to the end of the container.
    The element will end up at the end of the vector (analogue to a vector push_back!).
    What also happens is an insert into the hash table.

    If the key already exists in the Container, it will analogue to the HashTable
    replace the already existing data element in the vector.

    Identical to the insert() method

    Complexity: O(HashTable::find()+HashTable::insert()+Vector::push_back())

    \return iterator to the inserted pair (the last element if key did not exist before).
    */
    iterator push_back(const KeyT& key, const DataT& data)
    {
      agxAssert( checkSize() );

      typename HashType::iterator it = m_hash.find(key);
      if (it != m_hash.end())
      {
        // Replace the already existing data value with the new
        m_vector[it->second].second = data;
        return &m_vector[it->second];
      }

      // Key did not exist, insert at the end of the vector
      m_vector.push_back( std::make_pair(key, data) );

      // Hash on data, index is data value
      m_hash.insert(key, m_vector.size()-1);

      return end()-1; // Return iterator to the newly inserted element.
    }

    /**
    Insert a key,data pair in to the end of the container.
    The element will end up at the end of the vector (analogue to a vector push_back!).
    What also happens is an insert into the hash table.

    Identical to the push_back() method

    \verbatim Complexity: O(HashTable::find()+HashTable::insert()+Vector::push_back()) \endverbatim

    \return iterator to the inserted pair (at the end of the vector)
    */
    iterator insert(const KeyT& key, const DataT& data)
    {
      agxAssert( checkSize() );

      return push_back(key, data);
    }

    /**
    Replace data in hash vector.

    Similar to erase + insert but preserve iteration order.

    \note If \p oldKey doesn't exist, the operation is aborted and end() is returned.
    \param oldKey - old key (fails if not present in this hash vector)
    \param newKey - new key for data
    \param data - data for \p newKey
    */
    iterator replace( const KeyT& oldKey, const KeyT& newKey, const DataT& data )
    {
      agxAssert( checkSize() );
      auto oldIt = m_hash.find( oldKey );
      if ( oldIt == m_hash.end() )
        return end();

      size_t index = oldIt->second;
      m_vector[ index ] = std::make_pair( newKey, data );
      m_hash.erase( oldIt );
      m_hash.insert( newKey, index );

      return begin() + index;
    }

    /**
    Hash search for the key in the container.

    Complexity: O(HashTable::contains())


    \return true if key exists in the container.
    */
    bool contains(const KeyT& key) const
    {
      agxAssert( checkSize() );

      return m_hash.contains( key );
    }

    /**
    Perform a hash search for a given key.

    Complexity: O(HashTable::find())


    \return iterator to the found element, end() if none is found.
    */
    iterator find(const KeyT& key)
    {
      agxAssert( checkSize() );

      typename HashType::iterator it = m_hash.find( key );
      if (it == m_hash.end())
        return end();

      return &m_vector[it->second];
    }

    /**
    Perform a hash search for a given key.

    Complexity: O(HashTable::find())

    \return const_iterator to the found element, end() if none is found.
    */
    const_iterator find(const KeyT& key) const
    {
      agxAssert( checkSize() );

      typename HashType::const_iterator it = m_hash.find( key );
      if (it == m_hash.end())
        return end();

      // Return index to the element in the vector
      return &m_vector[it->second];

    }


    /**
    \return iterator to first element in the vector container
    */
    iterator begin()
    {
      agxAssert( checkSize() );


      return m_vector.begin(); }

    /**
    \return const_iterator to first element in the vector container
    */
    const_iterator begin() const
    {
      agxAssert( checkSize() );
      return m_vector.begin();
    }


    /**
    \return iterator marking end of the vector container
    */
    iterator end()
    {
      agxAssert( checkSize() );
      return m_vector.end();
    }

    /**
    \return const_iterator marking end of the vector container
    */
    const_iterator end() const
    {
      agxAssert( checkSize() );
      return m_vector.end();
    }

     /**
    \return a reference to the first element in the vector container
    */
    DataT& front()
    {
      agxAssert( checkSize() );
      return m_vector[0].second;
    }

   /**
    \return a const reference to the first element in the vector container
    */
    const DataT& front() const
    {
      agxAssert( checkSize() );
      return m_vector[0].second;
    }

    /**
    \return a reference to the last element in the vector container
    */
    DataT& back()
    {
      agxAssert( checkSize() );
      return m_vector[m_vector.size()-1].second;
    }

    /**
    \return a const reference to the last element in the vector container
    */
    const DataT& back() const
    {
      agxAssert( checkSize() );
      return m_vector[m_vector.size()-1].second;
    }



    /**
    Erase an element from the container.
    The elements in the vector CAN change order due to the use
    of Vector::fastErase, where the erased and the last element swap places.

    Complexity: O(HashTable::find() + HashTable::erase() + Vector::fastErase() )

    \return false if element was not found
    */
    bool erase(const KeyT& key)
    {

      agxAssert( checkSize() );

      typename HashType::iterator it = m_hash.find( key );
      if (it == m_hash.end())
        return false;

      erase( it );
      return true;
    }

    /**
    Erase an element using an iterator. This is a fastErase method where the
    erased element is replaced by the last elment.

    This method assumes that it is a valid/existing iterator in the container.
    If it is not, there is a chance that the iterator is point at something we do not want to
    touch, hence could cause crash. it is checked to see if it is == end(), then the method will return false.
    For other invalid iterators, behavior is undefined.

    Complexity: O(HashTable::find() + HashTable::erase() + Vector::fastErase() )

    \param it - Iterator to the element to be removed.
    \return the iterator to the position of the erased element where
            the last element is put if successful, otherwise end().
    */
    iterator erase(const iterator& it)
    {
      agxAssert( checkSize() );

      if (it == end())
        return it;

      KeyT& key = it->first;
      typename HashType::iterator hashIt = m_hash.find( key );

      agxAssert ( hashIt != m_hash.end() );

      hashIt = erase( hashIt );

      if ( hashIt == m_hash.end() )
        return end();

      return &m_vector[ hashIt->second ];
    }

    /**
    Remove all elements.
    \param policy - specifies the erase policy.
    */
    void clear(ClearPolicy policy = SHRINK_BUFFER_AVERAGED)
    {
      agxAssert( checkSize() );

      m_hash.clear( (typename HashType::ClearPolicy)policy );
      m_vector.clear( (typename VectorType::ClearPolicy)policy );
    }

    /**
    Access an element in the vector using an index
    \return reference to the Data in the i:th element in the vector
    */
    DataT& operator[](size_t i )
    {
      agxAssert( checkSize() );

      agxAssert(i < m_vector.size());
      return m_vector[i].second;
    }

    /**
    Access an element in the vector using an index
    \return const reference to the Data in the i:th element in the vector
    */
    const DataT& operator[](size_t i ) const
    {
      agxAssert( checkSize() );

      agxAssert(i < m_vector.size());
      return m_vector[i].second;
    }

    /**
    Reserve capacity in the hash table.
    \param size - reserve size elements
    */
    void reserve(size_t size)
    {
      agxAssert( checkSize() );

      m_vector.reserve( size );
      m_hash.reserve( size );
    }

    /**
    Returns value given key. If the key doesn't exist, the key
    is inserted and the default value is returned.
    \param key - key
    \return data for key
    */
    DataT& getOrCreate( const KeyT& key )
    {
      iterator it = find( key );
      if ( it == end() )
        it = insert( key, value_type() );
      return it->second;
    }

    /**
    Purge the hash set. Good to use when removing multiple items, since iterators
    are invalidated after any modification. The purger should be a functor class
    which takes the key as parameter and return true if the entry should
    be removed from the hash set.

    Complexity: n - number of purged elements O(n*HashTable::erase())

    Example of a purger implementation:

    template < typename T1, typename T2 >
    class MyPurger
    {
    public:
      MyPurger() {}
      inline bool operator()(const T& a)
      {
        return a==2;
      }
    };

    \param purger - An instance of a class with an bool () operator which when it returns true, the element will be removed
    from the container
    \return number of purged elements
    */
    template<typename T>
    size_t purge(T purger)
    {
      if (empty()) // Nothing to do
        return 0;

#if 0 // Slow version
      VectorType savedElements; // Elements to keep
      savedElements.reserve( m_vector.size());
      VectorType::const_iterator it = m_vector.begin();
      for(; it != m_vector.end(); ++it)
      {
        // Should we keep it?
        if (!purger(it->first, it->second))
          savedElements.push_back( std::make_pair(it->first, it->second)); // store all elements that should remain
      }
      m_vector.clear(agx::Container::SHRINK_BUFFER_AVERAGED);
      m_hash.clear(agx::Container::SHRINK_BUFFER_AVERAGED);

      // Make sure we can fit all the saved elements
      m_vector.reserve( savedElements.size() );
      m_hash.reserve( savedElements.size() );

      for(VectorType::const_iterator it = savedElements.begin(); it != savedElements.end(); ++it)
      {
        m_vector.push_back( *it );
        // Hash on data, index is data value
        m_hash.insert( it->first, m_vector.size()-1 );
      }
#else // Faster version (2x)
      typename agx::Vector<typename VectorType::value_type *> removeElements; //PointerVector ; // Safe to use pointers?
      removeElements.reserve( m_vector.size());
      for(typename VectorType::iterator it = m_vector.begin(); it != m_vector.end(); ++it)
      {
        // Should we keep it?
        if (!purger((*it).first, (*it).second ))
          removeElements.push_back(&(*it)); // store all elements that should remain
      }

      for(typename agx::Vector<typename VectorType::value_type *>::const_iterator it = removeElements.begin(); it != removeElements.end(); ++it)
      {
        this->erase( (**it).first );
      }

      return removeElements.size();
#endif
    }

  private:

    // Return true if size of vector and hash is equal
    bool checkSize() const {
      return m_vector.size() == m_hash.size();
    }

    typename HashType::iterator erase( typename HashType::iterator it )
    {
      size_t idx = it->second;

      typename HashType::iterator retIt = m_hash.erase( it ); // remove from hash

      // Remove from vector too:
      if (m_vector.size() > 1 && idx != (m_vector.size()-1))
      {
        // Get the last element
        typename VectorType::value_type lastElement = m_vector.back();

        // Put it where we have removed an element
        m_vector[idx] = lastElement;

        // remove the last now
        m_vector.pop_back();

        // Rehash the moved one with the new index
        retIt = m_hash.insert( lastElement.first, idx );
      }
      else
        // Only one element, OR its the last we are removing: just remove it
        m_vector.pop_back();

      agxAssert( checkSize() );

      return retIt;
    }


  };

}


#endif /* AGX_HASHVECTOR_H */
