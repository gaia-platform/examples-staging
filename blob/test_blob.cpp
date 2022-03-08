/////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
// All rights reserved.
/////////////////////////////////////////////


#include <cstring>

#include <iostream>

#include "blob.hpp"

using namespace std;

void TestBlob();

int main(void)
{
    TestBlob();
}

void TestBlob()
{
    blob_t* blob1_ptr = blob_t::create_blob(1, 1, 0);
    blob1_ptr->set_state(1);

    blob_t* blob2_ptr = blob_t::create_blob(2, 1, 1);
    blob2_ptr->set_state(2);

    blob_t* blob_ptr = blob_t::get_blob(1);
    cout << "Blob 1 has state '" << blob_ptr->get_state() << "'" << endl;

    blob_ptr = blob_t::get_blob(2);
    cout << "Blob 2 has state '" << blob_ptr->get_state() << "'" << endl;

    blob_t* blob3_ptr = blob_t::create_blob(3, 1, 2);
    blob3_ptr->set_state(3);

    blob_ptr = blob_t::get_blob(3);
    cout << "Blob 3 has state '" << blob_ptr->get_state() << "'" << endl;

    blob_ptr = blob_t::get_blob(1);
    if (blob_ptr)
    {
        cout << "FAILED: Blob 1 still exists and has state '" << blob_ptr->get_state() << "'" << endl;
    }
    else
    {
        cout << "Blob 1 has been removed, as expected!" << endl;
    }
}
