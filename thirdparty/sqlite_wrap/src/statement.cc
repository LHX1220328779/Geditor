

#include "statement.h"

#include "database.h"
#include "column.h"
#include "exception.h"

#include "sqlite3.h"

namespace SQLite
{

// Compile and register the SQL query for the provided SQLite Database Connection
Statement::Statement(Database &aDatabase, const char* apQuery) :
    query_(apQuery),
    stmt_ptr_(aDatabase.sqLite_ptr_, query_), // prepare the SQL query, and ref count (needs Database friendship)
    column_count_(0),
    if_row_fetched_(false),
    last_executestep_done_(false)
{
    column_count_ = sqlite3_column_count(stmt_ptr_);
}

// Compile and register the SQL query for the provided SQLite Database Connection
Statement::Statement(Database &aDatabase, const std::string& aQuery) :
    query_(aQuery),
    stmt_ptr_(aDatabase.sqLite_ptr_, query_), // prepare the SQL query, and ref count (needs Database friendship)
    column_count_(0),
    if_row_fetched_(false),
    last_executestep_done_(false)
{
    column_count_ = sqlite3_column_count(stmt_ptr_);
}


// Finalize and unregister the SQL query from the SQLite Database Connection.
Statement::~Statement()
{
    // the finalization will be done by the destructor of the last shared pointer
}

// Reset the statement to make it ready for a new execution (see also #clearBindings() bellow)
void Statement::reset()
{
    const int ret = tryReset();
    check(ret);
}

int Statement::tryReset() 
{
    if_row_fetched_ = false;
    last_executestep_done_ = false;
    return sqlite3_reset(stmt_ptr_);
}

// Clears away all the bindings of a prepared statement (can be associated with #reset() above).
void Statement::clearBindings()
{
    const int ret = sqlite3_clear_bindings(stmt_ptr_);
    check(ret);
}

// Bind an int value to a parameter "?", "?NNN", ":VVV", "@VVV" or "$VVV" in the SQL prepared statement
void Statement::bind(const int aIndex, const int aValue)
{
    const int ret = sqlite3_bind_int(stmt_ptr_, aIndex, aValue);
    check(ret);
}

// Bind a 32bits unsigned int value to a parameter "?", "?NNN", ":VVV", "@VVV" or "$VVV" in the SQL prepared statement
void Statement::bind(const int aIndex, const unsigned aValue)
{
    const int ret = sqlite3_bind_int64(stmt_ptr_, aIndex, aValue);
    check(ret);
}

// Bind a 64bits int value to a parameter "?", "?NNN", ":VVV", "@VVV" or "$VVV" in the SQL prepared statement
void Statement::bind(const int aIndex, const long long aValue)
{
    const int ret = sqlite3_bind_int64(stmt_ptr_, aIndex, aValue);
    check(ret);
}

// Bind a double (64bits float) value to a parameter "?", "?NNN", ":VVV", "@VVV" or "$VVV" in the SQL prepared statement
void Statement::bind(const int aIndex, const double aValue)
{
    const int ret = sqlite3_bind_double(stmt_ptr_, aIndex, aValue);
    check(ret);
}

// Bind a string value to a parameter "?", "?NNN", ":VVV", "@VVV" or "$VVV" in the SQL prepared statement
void Statement::bind(const int aIndex, const std::string& aValue)
{
    const int ret = sqlite3_bind_text(stmt_ptr_, aIndex, aValue.c_str(),
                                      static_cast<int>(aValue.size()), SQLITE_TRANSIENT);
    check(ret);
}

// Bind a text value to a parameter "?", "?NNN", ":VVV", "@VVV" or "$VVV" in the SQL prepared statement
void Statement::bind(const int aIndex, const char* apValue)
{
    const int ret = sqlite3_bind_text(stmt_ptr_, aIndex, apValue, -1, SQLITE_TRANSIENT);
    check(ret);
}

// Bind a binary blob value to a parameter "?", "?NNN", ":VVV", "@VVV" or "$VVV" in the SQL prepared statement
void Statement::bind(const int aIndex, const void* apValue, const int aSize)
{
    const int ret = sqlite3_bind_blob(stmt_ptr_, aIndex, apValue, aSize, SQLITE_TRANSIENT);
    check(ret);
}

// Bind a string value to a parameter "?", "?NNN", ":VVV", "@VVV" or "$VVV" in the SQL prepared statement
void Statement::bindNoCopy(const int aIndex, const std::string& aValue)
{
    const int ret = sqlite3_bind_text(stmt_ptr_, aIndex, aValue.c_str(),
                                      static_cast<int>(aValue.size()), SQLITE_STATIC);
    check(ret);
}

// Bind a text value to a parameter "?", "?NNN", ":VVV", "@VVV" or "$VVV" in the SQL prepared statement
void Statement::bindNoCopy(const int aIndex, const char* apValue)
{
    const int ret = sqlite3_bind_text(stmt_ptr_, aIndex, apValue, -1, SQLITE_STATIC);
    check(ret);
}

// Bind a binary blob value to a parameter "?", "?NNN", ":VVV", "@VVV" or "$VVV" in the SQL prepared statement
void Statement::bindNoCopy(const int aIndex, const void* apValue, const int aSize)
{
    const int ret = sqlite3_bind_blob(stmt_ptr_, aIndex, apValue, aSize, SQLITE_STATIC);
    check(ret);
}

// Bind a NULL value to a parameter "?", "?NNN", ":VVV", "@VVV" or "$VVV" in the SQL prepared statement
void Statement::bind(const int aIndex)
{
    const int ret = sqlite3_bind_null(stmt_ptr_, aIndex);
    check(ret);
}


// Bind an int value to a parameter "?NNN", ":VVV", "@VVV" or "$VVV" in the SQL prepared statement
void Statement::bind(const char* apName, const int aValue)
{
    const int index = sqlite3_bind_parameter_index(stmt_ptr_, apName);
    const int ret = sqlite3_bind_int(stmt_ptr_, index, aValue);
    check(ret);
}

// Bind a 32bits unsigned int value to a parameter "?NNN", ":VVV", "@VVV" or "$VVV" in the SQL prepared statement
void Statement::bind(const char* apName, const unsigned aValue)
{
    const int index = sqlite3_bind_parameter_index(stmt_ptr_, apName);
    const int ret = sqlite3_bind_int64(stmt_ptr_, index, aValue);
    check(ret);
}

// Bind a 64bits int value to a parameter "?NNN", ":VVV", "@VVV" or "$VVV" in the SQL prepared statement
void Statement::bind(const char* apName, const long long aValue)
{
    const int index = sqlite3_bind_parameter_index(stmt_ptr_, apName);
    const int ret = sqlite3_bind_int64(stmt_ptr_, index, aValue);
    check(ret);
}

// Bind a double (64bits float) value to a parameter "?NNN", ":VVV", "@VVV" or "$VVV" in the SQL prepared statement
void Statement::bind(const char* apName, const double aValue)
{
    const int index = sqlite3_bind_parameter_index(stmt_ptr_, apName);
    const int ret = sqlite3_bind_double(stmt_ptr_, index, aValue);
    check(ret);
}

// Bind a string value to a parameter "?NNN", ":VVV", "@VVV" or "$VVV" in the SQL prepared statement
void Statement::bind(const char* apName, const std::string& aValue)
{
    const int index = sqlite3_bind_parameter_index(stmt_ptr_, apName);
    const int ret = sqlite3_bind_text(stmt_ptr_, index, aValue.c_str(),
                                      static_cast<int>(aValue.size()), SQLITE_TRANSIENT);
    check(ret);
}

// Bind a text value to a parameter "?NNN", ":VVV", "@VVV" or "$VVV" in the SQL prepared statement
void Statement::bind(const char* apName, const char* apValue)
{
    const int index = sqlite3_bind_parameter_index(stmt_ptr_, apName);
    const int ret = sqlite3_bind_text(stmt_ptr_, index, apValue, -1, SQLITE_TRANSIENT);
    check(ret);
}

// Bind a binary blob value to a parameter "?NNN", ":VVV", "@VVV" or "$VVV" in the SQL prepared statement
void Statement::bind(const char* apName, const void* apValue, const int aSize)
{
    const int index = sqlite3_bind_parameter_index(stmt_ptr_, apName);
    const int ret = sqlite3_bind_blob(stmt_ptr_, index, apValue, aSize, SQLITE_TRANSIENT);
    check(ret);
}

// Bind a string value to a parameter "?NNN", ":VVV", "@VVV" or "$VVV" in the SQL prepared statement
void Statement::bindNoCopy(const char* apName, const std::string& aValue)
{
    const int index = sqlite3_bind_parameter_index(stmt_ptr_, apName);
    const int ret = sqlite3_bind_text(stmt_ptr_, index, aValue.c_str(),
                                      static_cast<int>(aValue.size()), SQLITE_STATIC);
    check(ret);
}

// Bind a text value to a parameter "?NNN", ":VVV", "@VVV" or "$VVV" in the SQL prepared statement
void Statement::bindNoCopy(const char* apName, const char* apValue)
{
    const int index = sqlite3_bind_parameter_index(stmt_ptr_, apName);
    const int ret = sqlite3_bind_text(stmt_ptr_, index, apValue, -1, SQLITE_STATIC);
    check(ret);
}

// Bind a binary blob value to a parameter "?NNN", ":VVV", "@VVV" or "$VVV" in the SQL prepared statement
void Statement::bindNoCopy(const char* apName, const void* apValue, const int aSize)
{
    const int index = sqlite3_bind_parameter_index(stmt_ptr_, apName);
    const int ret = sqlite3_bind_blob(stmt_ptr_, index, apValue, aSize, SQLITE_STATIC);
    check(ret);
}

// Bind a NULL value to a parameter "?NNN", ":VVV", "@VVV" or "$VVV" in the SQL prepared statement
void Statement::bind(const char* apName)
{
    const int index = sqlite3_bind_parameter_index(stmt_ptr_, apName);
    const int ret = sqlite3_bind_null(stmt_ptr_, index);
    check(ret);
}


// Execute a step of the query to fetch one row of results
bool Statement::executeStep()
{
    const int ret = tryExecuteStep();
    if ((SQLITE_ROW != ret) && (SQLITE_DONE != ret)) // on row or no (more) row ready, else it's a problem
    {
       // throw SQLite::Exception(stmt_ptr_, ret);
    }

    return if_row_fetched_; // true only if one row is accessible by getColumn(N)
}

// Execute a one-step query with no expected result
int Statement::exec()
{
    const int ret = tryExecuteStep();
    if (SQLITE_DONE != ret) // the statement has finished executing successfully
    {
        if (SQLITE_ROW == ret)
        {
            //throw SQLite::Exception("exec() does not expect results. Use executeStep.");
        }
        else
        {
            //throw SQLite::Exception(stmt_ptr_, ret);
        }
		return 0;
    }

    // Return the number of rows modified by those SQL statements (INSERT, UPDATE or DELETE)
    return sqlite3_changes(stmt_ptr_);
}

int Statement::tryExecuteStep() 
{
    if (false == last_executestep_done_)
    {
        const int ret = sqlite3_step(stmt_ptr_);
        if (SQLITE_ROW == ret) // one row is ready : call getColumn(N) to access it
        {
            if_row_fetched_ = true;
        }
        else if (SQLITE_DONE == ret) // no (more) row ready : the query has finished executing
        {
            if_row_fetched_ = false;
            last_executestep_done_ = true;
        }
        else
        {
            if_row_fetched_ = false;
            last_executestep_done_ = false;
        }

        return ret;
    }
    else
    {
        // Statement needs to be reseted !
        return SQLITE_MISUSE;
    }
}


// Return a copy of the column data specified by its index starting at 0
// (use the Column copy-constructor)
Column Statement::getColumn(const int aIndex)
{
    checkRow();
    checkIndex(aIndex);

    // Share the Statement Object handle with the new Column created
    return Column(stmt_ptr_, aIndex);
}

// Return a copy of the column data specified by its column name starting at 0
// (use the Column copy-constructor)
Column  Statement::getColumn(const char* apName)
{
    checkRow();
    const int index = getColumnIndex(apName);

    // Share the Statement Object handle with the new Column created
    return Column(stmt_ptr_, index);
}

// Test if the column is NULL
bool Statement::isColumnNull(const int aIndex) const
{
    checkRow();
    checkIndex(aIndex);
    return (SQLITE_NULL == sqlite3_column_type(stmt_ptr_, aIndex));
}

bool Statement::isColumnNull(const char* apName) const
{
    checkRow();
    const int index = getColumnIndex(apName);
    return (SQLITE_NULL == sqlite3_column_type(stmt_ptr_, index));
}

// Return the named assigned to the specified result column (potentially aliased)
const char* Statement::getColumnName(const int aIndex) const
{
    checkIndex(aIndex);
    return sqlite3_column_name(stmt_ptr_, aIndex);
}

#ifdef SQLITE_ENABLE_COLUMN_METADATA
// Return the named assigned to the specified result column (potentially aliased)
const char* Statement::getColumnOriginName(const int aIndex) const
{
    checkIndex(aIndex);
    return sqlite3_column_origin_name(stmt_ptr_, aIndex);
}
#endif

// Return the index of the specified (potentially aliased) column name
int Statement::getColumnIndex(const char* apName) const
{
    // Build the map of column index by name on first call
    if (column_names_.empty())
    {
        for (int i = 0; i < column_count_; ++i)
        {
            const char* pName = sqlite3_column_name(stmt_ptr_, i);
            column_names_[pName] = i;
        }
    }

    const TColumnNames::const_iterator iIndex = column_names_.find(apName);
    if (iIndex == column_names_.end())
    {
        //throw SQLite::Exception("Unknown column name.");
    }

    return (*iIndex).second;
}

// Return the numeric result code for the most recent failed API call (if any).
int Statement::GetErrorCode() const  // nothrow
{
    return sqlite3_errcode(stmt_ptr_);
}
// Return the extended numeric result code for the most recent failed API call (if any).
int Statement::GetExtendedErrorCode() const  // nothrow
{
    return sqlite3_extended_errcode(stmt_ptr_);
}
// Return UTF-8 encoded English language explanation of the most recent failed API call (if any).
const char* Statement::getErrorMsg() const  // nothrow
{
    return sqlite3_errmsg(stmt_ptr_);
}

////////////////////////////////////////////////////////////////////////////////
// Internal class : shared pointer to the sqlite3_stmt SQLite Statement Object
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Prepare the statement and initialize its reference counter
 *
 * @param[in] apSQLite  The sqlite3 database connexion
 * @param[in] aQuery    The SQL query string to prepare
 */
Statement::Ptr::Ptr(sqlite3* apSQLite, std::string& aQuery) :
    sqLite_ptr_(apSQLite),
    sqlite_stmt_pointer_(NULL),
    ref_count_pointer_(NULL)
{
    const int ret = sqlite3_prepare_v2(apSQLite, aQuery.c_str(), static_cast<int>(aQuery.size()), &sqlite_stmt_pointer_, NULL);
    if (SQLITE_OK != ret)
    {
        //throw SQLite::Exception(apSQLite, ret);
		bool resh =0;
    }
    // Initialize the reference counter of the sqlite3_stmt :
    // used to share the stmt_ptr_ between Statement and Column objects;
    // This is needed to enable Column objects to live longer than the Statement objet it refers to.
    ref_count_pointer_ = new unsigned int(1);  // NOLINT(readability/casting)
}

/**
 * @brief Copy constructor increments the ref counter
 *
 * @param[in] aPtr Pointer to copy
 */
Statement::Ptr::Ptr(const Statement::Ptr& aPtr) :
    sqLite_ptr_(aPtr.sqLite_ptr_),
    sqlite_stmt_pointer_(aPtr.sqlite_stmt_pointer_),
    ref_count_pointer_(aPtr.ref_count_pointer_)
{
    //assert(NULL != ref_count_pointer_);
    //assert(0 != *ref_count_pointer_);

    // Increment the reference counter of the sqlite3_stmt,
    // asking not to finalize the sqlite3_stmt during the lifetime of the new objet
    ++(*ref_count_pointer_);
}

/**
 * @brief Decrement the ref counter and finalize the sqlite3_stmt when it reaches 0
 */
Statement::Ptr::~Ptr()
{
    //assert(NULL != ref_count_pointer_);
    //assert(0 != *ref_count_pointer_);

    // Decrement and check the reference counter of the sqlite3_stmt
    --(*ref_count_pointer_);
    if (0 == *ref_count_pointer_)
    {
        // If count reaches zero, finalize the sqlite3_stmt, as no Statement nor Column objet use it anymore.
        // No need to check the return code, as it is the same as the last statement evaluation.
        sqlite3_finalize(sqlite_stmt_pointer_);

        // and delete the reference counter
        delete ref_count_pointer_;
        ref_count_pointer_ = NULL;
        sqlite_stmt_pointer_ = NULL;
    }
    // else, the finalization will be done later, by the last object
}


}  // namespace SQLite
