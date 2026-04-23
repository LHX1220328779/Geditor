

#include "column.h"

#include "sqlite3.h"

namespace SQLite
{

const int INTEGER   = SQLITE_INTEGER;
const int FLOAT     = SQLITE_FLOAT;
const int TEXT      = SQLITE_TEXT;
const int BLOB      = SQLITE_BLOB;
const int Null      = SQLITE_NULL;


// Encapsulation of a Column in a row of the result pointed by the prepared Statement.
Column::Column(Statement::Ptr& aStmtPtr, int aIndex)  : // nothrow
    stmt_ptr_(aStmtPtr),
    mIndex(aIndex)
{
}

// Finalize and unregister the SQL query from the SQLite Database Connection.
Column::~Column()
{
    // the finalization will be done by the destructor of the last shared pointer
}

// Return the named assigned to this result column (potentially aliased)
const char* Column::getName() const  // nothrow
{
    return sqlite3_column_name(stmt_ptr_, mIndex);
}

#ifdef SQLITE_ENABLE_COLUMN_METADATA
// Return the name of the table column that is the origin of this result column
const char* Column::getOriginName() const  // nothrow
{
    return sqlite3_column_origin_name(stmt_ptr_, mIndex);
}
#endif

// Return the integer value of the column specified by its index starting at 0
int Column::getInt() const  // nothrow
{
    return sqlite3_column_int(stmt_ptr_, mIndex);
}

// Return the unsigned integer value of the column specified by its index starting at 0
unsigned Column::getUInt() const  // nothrow
{
    return static_cast<unsigned>(getInt64());
}

// Return the 64bits integer value of the column specified by its index starting at 0
long long Column::getInt64() const  // nothrow
{
    return sqlite3_column_int64(stmt_ptr_, mIndex);
}

// Return the double value of the column specified by its index starting at 0
double Column::getDouble() const  // nothrow
{
    return sqlite3_column_double(stmt_ptr_, mIndex);
}

// Return a pointer to the text value (NULL terminated string) of the column specified by its index starting at 0
const char* Column::getText() const  // nothrow
{
    const char* pText = reinterpret_cast<const char*>(sqlite3_column_text(stmt_ptr_, mIndex));
    return pText;
}

// Return a pointer to the blob value (*not* NULL terminated) of the column specified by its index starting at 0
const void* Column::getBlob() const  // nothrow
{
    return sqlite3_column_blob(stmt_ptr_, mIndex);
}

// Return a std::string to a TEXT or BLOB column
std::string Column::getString() const
{
    // Note: using sqlite3_column_blob and not sqlite3_column_text
    // - no need for sqlite3_column_text to add a \0 on the end, as we're getting the bytes length directly
    const char *data = static_cast<const char *>(sqlite3_column_blob(stmt_ptr_, mIndex));

    // SQLite docs: "The safest policy is to invoke… sqlite3_column_blob() followed by sqlite3_column_bytes()"
    // Note: std::string is ok to pass nullptr as first arg, if length is 0
    return std::string(data, sqlite3_column_bytes(stmt_ptr_, mIndex));
}

// Return the type of the value of the column
int Column::getType() const  // nothrow
{
    return sqlite3_column_type(stmt_ptr_, mIndex);
}

// Return the number of bytes used by the text value of the column
int Column::getBytes() const  // nothrow
{
    return sqlite3_column_bytes(stmt_ptr_, mIndex);
}



}  // namespace SQLite
