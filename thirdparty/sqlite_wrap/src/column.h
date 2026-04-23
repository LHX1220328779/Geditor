

#pragma once

#include "statement.h"
#include "exception.h"

#include <string>
#include <limits.h>

namespace SQLite
{

extern const int INTEGER;   ///< SQLITE_INTEGER
extern const int FLOAT;     ///< SQLITE_FLOAT
extern const int TEXT;      ///< SQLITE_TEXT
extern const int BLOB;      ///< SQLITE_BLOB
extern const int Null;      ///< SQLITE_NULL


/**
 * @brief Encapsulation of a Column in a row of the result pointed by the prepared Statement.
 *
 *  A Column is a particular field of SQLite data in the current row of result
 * of the Statement : it points to a single cell.
 *
 * Its value can be expressed as a text, and, when applicable, as a numeric
 * (integer or floating point) or a binary blob.
 *
 * Thread-safety: a Column object shall not be shared by multiple threads, because :
 * 1) in the SQLite "Thread Safe" mode, "SQLite can be safely used by multiple threads
 *    provided that no single database connection is used simultaneously in two or more threads."
 * 2) the SQLite "Serialized" mode is not supported by SQLiteC++,
 *    because of the way it shares the underling SQLite precompiled statement
 *    in a custom shared pointer (See the inner class "Statement::Ptr").
 */
class Column
{
public:
    /**
     * @brief Encapsulation of a Column in a Row of the result.
     *
     * @param[in] aStmtPtr  Shared pointer to the prepared SQLite Statement Object.
     * @param[in] aIndex    Index of the column in the row of result, starting at 0
     */
    Column(Statement::Ptr& aStmtPtr, int aIndex)    ; // nothrow
    /// Simple destructor
    ~Column();

    // default copy constructor and assignment operator are perfectly suited :
    // they copy the Statement::Ptr which in turn increments the reference counter.

    /// Make clang happy by explicitly implementing the copy-constructor:
    Column(const Column & aOther) :
        stmt_ptr_(aOther.stmt_ptr_),
        mIndex(aOther.mIndex)
    {
    }

    /**
     * @brief Return a pointer to the named assigned to this result column (potentially aliased)
     *
     * @see getOriginName() to get original column name (not aliased)
     */
    const char* getName() const ; // nothrow

#ifdef SQLITE_ENABLE_COLUMN_METADATA
    /**
     * @brief Return a pointer to the table column name that is the origin of this result column
     * 
     *  Require definition of the SQLITE_ENABLE_COLUMN_METADATA preprocessor macro :
     * - when building the SQLite library itself (which is the case for the Debian libsqlite3 binary for instance),
     * - and also when compiling this wrapper.
     */
    const char* getOriginName() const ; // nothrow
#endif

    /// Return the integer value of the column.
    int         getInt() const ; // nothrow
    /// Return the 32bits unsigned integer value of the column (note that SQLite3 does not support unsigned 64bits).
    unsigned    getUInt() const ; // nothrow
    /// Return the 64bits integer value of the column (note that SQLite3 does not support unsigned 64bits).
    long long   getInt64() const ; // nothrow
    /// Return the double (64bits float) value of the column
    double      getDouble() const ; // nothrow
    /**
     * @brief Return a pointer to the text value (NULL terminated string) of the column.
     *
     * @warning The value pointed at is only valid while the statement is valid (ie. not finalized),
     *          thus you must copy it before using it beyond its scope (to a std::string for instance).
     */
    const char* getText() const ; // nothrow
    /**
     * @brief Return a pointer to the binary blob value of the column.
     *
     * @warning The value pointed at is only valid while the statement is valid (ie. not finalized),
     *          thus you must copy it before using it beyond its scope (to a std::string for instance).
     */
    const void* getBlob() const ; // nothrow
    /**
     * @brief Return a std::string for a TEXT or BLOB column.
     *
     * Note this correctly handles strings that contain null bytes.
     */
    std::string getString() const;

    /**
     * @brief Return the type of the value of the column
     *
     * Return either SQLite::INTEGER, SQLite::FLOAT, SQLite::TEXT, SQLite::BLOB, or SQLite::Null.
     *
     * @warning After a type conversion (by a call to a getXxx on a Column of a Yyy type),
     *          the value returned by sqlite3_column_type() is undefined.
     */
    int getType() const ; // nothrow

    /// Test if the column is an integer type value (meaningful only before any conversion)
    inline bool isInteger() const  // nothrow
    {
        return (SQLite::INTEGER == getType());
    }
    /// Test if the column is a floating point type value (meaningful only before any conversion)
    inline bool isFloat() const  // nothrow
    {
        return (SQLite::FLOAT == getType());
    }
    /// Test if the column is a text type value (meaningful only before any conversion)
    inline bool isText() const  // nothrow
    {
        return (SQLite::TEXT == getType());
    }
    /// Test if the column is a binary blob type value (meaningful only before any conversion)
    inline bool isBlob() const  // nothrow
    {
        return (SQLite::BLOB == getType());
    }
    /// Test if the column is NULL (meaningful only before any conversion)
    inline bool isNull() const  // nothrow
    {
        return (SQLite::Null == getType());
    }

    /**
     * @brief Return the number of bytes used by the text (or blob) value of the column
     *
     * Return either :
     * - size in bytes (not in characters) of the string returned by getText() without the '\0' terminator
     * - size in bytes of the string representation of the numerical value (integer or double)
     * - size in bytes of the binary blob returned by getBlob()
     * - 0 for a NULL value
     */
    int getBytes() const ;

    /// Alias returning the number of bytes used by the text (or blob) value of the column
    inline int size() const 
    {
        return getBytes ();
    }

    /// Inline cast operator to int
    inline operator int() const
    {
        return getInt();
    }
    /// Inline cast operator to 32bits unsigned integer
    inline operator unsigned int() const
    {
        return getUInt();
    }
#if (LONG_MAX == INT_MAX) // sizeof(long)==4 means the data model of the system is ILP32 (32bits OS or Windows 64bits)
    /// Inline cast operator to 32bits long
    inline operator long() const
    {
        return getInt();
    }
    /// Inline cast operator to 32bits unsigned long
    inline operator unsigned long() const
    {
        return getUInt();
    }
#else
    /// Inline cast operator to 64bits long when the data model of the system is ILP64 (Linux 64 bits...)
    inline operator long() const
    {
        return getInt64();
    }
#endif

    /// Inline cast operator to 64bits integer
    inline operator long long() const
    {
        return getInt64();
    }
    /// Inline cast operator to double
    inline operator double() const
    {
        return getDouble();
    }
    /**
     * @brief Inline cast operator to char*
     *
     * @see getText
     */
    inline operator const char*() const
    {
        return getText();
    }
    /**
     * @brief Inline cast operator to void*
     *
     * @see getBlob
     */
    inline operator const void*() const
    {
        return getBlob();
    }

private:
    Statement::Ptr  stmt_ptr_;   ///< Shared Pointer to the prepared SQLite Statement Object
    int             mIndex;     ///< Index of the column in the row of result, starting at 0
};


#if __cplusplus >= 201402L || (defined(_MSC_VER) && _MSC_VER >= 1900)

// Create an instance of T from the first N columns, see declaration in Statement.h for full details
template<typename T, int N>
T Statement::getColumns()
{
    checkRow();
    checkIndex(N - 1);
    return getColumns<T>(std::make_integer_sequence<int, N>{});
}

// Helper function called by getColums<typename T, int N>
template<typename T, const int... Is>
T Statement::getColumns(const std::integer_sequence<int, Is...>)
{
    return T{Column(stmt_ptr_, Is)...};
}

#endif

}  // namespace SQLite
