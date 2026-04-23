

#include "exception.h"

#include "sqlite3.h"

namespace SQLite
{

Exception::Exception(const char* aErrorMessage) :
    err_code_(-1), // 0 would be SQLITE_OK, which doesn't make sense
    extended_errcode_(-1)
{
}
Exception::Exception(const std::string& aErrorMessage) :
    err_code_(-1), // 0 would be SQLITE_OK, which doesn't make sense
    extended_errcode_(-1)
{
}

Exception::Exception(const char* aErrorMessage, int ret) :
    err_code_(ret),
    extended_errcode_(-1)
{
}

Exception::Exception(const std::string& aErrorMessage, int ret) :
    err_code_(ret),
    extended_errcode_(-1)
{
}

Exception::Exception(sqlite3* apSQLite) :
    err_code_(sqlite3_errcode(apSQLite)),
    extended_errcode_(sqlite3_extended_errcode(apSQLite))
{
}

Exception::Exception(sqlite3* apSQLite, int ret) :
    err_code_(ret),
    extended_errcode_(sqlite3_extended_errcode(apSQLite))
{
}

// Return a string, solely based on the error code
const char* Exception::GetErrorStr() const  // nothrow
{
    return sqlite3_errstr(err_code_);
}


}  // namespace SQLite
