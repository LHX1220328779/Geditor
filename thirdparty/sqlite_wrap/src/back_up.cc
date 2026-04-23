
#include "back_up.h"

#include "exception.h"

#include "sqlite3.h"

namespace SQLite
{

// Initialize resource for SQLite database backup
Backup::Backup(Database&    aDestDatabase,
               const char*  apDestDatabaseName,
               Database&    aSrcDatabase,
               const char*  apSrcDatabaseName) :
    sqlite_backup_(NULL)
{
    sqlite_backup_ = sqlite3_backup_init(aDestDatabase.getHandle(),
                                         apDestDatabaseName,
                                         aSrcDatabase.getHandle(),
                                         apSrcDatabaseName);
    if (NULL == sqlite_backup_)
    {
        // If an error occurs, the error code and message are attached to the destination database connection.
        check(aDestDatabase.getHandle());
    }
}

// Initialize resource for SQLite database backup
Backup::Backup(Database&            aDestDatabase,
               const std::string&   aDestDatabaseName,
               Database&            aSrcDatabase,
               const std::string&   aSrcDatabaseName) :
    sqlite_backup_(NULL)
{
    sqlite_backup_ = sqlite3_backup_init(aDestDatabase.getHandle(),
                                         aDestDatabaseName.c_str(),
                                         aSrcDatabase.getHandle(),
                                         aSrcDatabaseName.c_str());
    if (NULL == sqlite_backup_)
    {
        // If an error occurs, the error code and message are attached to the destination database connection.
        check(aDestDatabase.getHandle());
    }
}

// Initialize resource for SQLite database backup
Backup::Backup(Database &aDestDatabase, Database &aSrcDatabase) :
    sqlite_backup_(NULL)
{
    sqlite_backup_ = sqlite3_backup_init(aDestDatabase.getHandle(),
                                         "main",
                                         aSrcDatabase.getHandle(),
                                         "main");
    if (NULL == sqlite_backup_)
    {
        // If an error occurs, the error code and message are attached to the destination database connection.
        check(aDestDatabase.getHandle());
    }
}

// Release resource for SQLite database backup
Backup::~Backup()
{
    if (NULL != sqlite_backup_)
    {
        sqlite3_backup_finish(sqlite_backup_);
    }
}

// Execute backup step with a given number of source pages to be copied
int Backup::executeStep(const int aNumPage /* = -1 */)
{
    const int res = sqlite3_backup_step(sqlite_backup_, aNumPage);
    if (SQLITE_OK != res && SQLITE_DONE != res && SQLITE_BUSY != res && SQLITE_LOCKED != res)
    {
        //throw SQLite::Exception(sqlite3_errstr(res), res);
    }
    return res;
}

// Get the number of remaining source pages to be copied in this backup process
int Backup::getRemainingPageCount()
{
    return sqlite3_backup_remaining(sqlite_backup_);
}

// Get the number of total source pages to be copied in this backup process
int Backup::getTotalPageCount()
{
    return sqlite3_backup_pagecount(sqlite_backup_);
}

}  // namespace SQLite
