
#include "transaction.h"

#include "database.h"

namespace SQLite
{


// Begins the SQLite transaction
Transaction::Transaction(Database& aDatabase) :
    database_ref_(aDatabase),
    if_commit_called_(false)
{
    database_ref_.exec("BEGIN");
}

// Safely rollback the transaction if it has not been committed.
Transaction::~Transaction()
{
    if (false == if_commit_called_)
    {
       // try
        {
            database_ref_.exec("ROLLBACK");
        }
        //catch (SQLite::Exception&)
        {
            // Never throw an exception in a destructor: error if already rollbacked, but no harm is caused by this.
        }
    }
}

// Commit the transaction.
void Transaction::commit()
{
    if (false == if_commit_called_)
    {
        database_ref_.exec("COMMIT");
        if_commit_called_ = true;
    }
    else
    {
        //throw SQLite::Exception("Transaction already commited.");
    }
}


}  // namespace SQLite
