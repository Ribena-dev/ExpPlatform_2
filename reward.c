#include <stdio.h>
#include <stdlib.h>
#include "hidapi.h"
#include "libmcp2221.h"

int main()
{
    int res;
    res = hid_init();
    handle = hid_open(0x4d8, 0x3f, NULL);
    res = hid_get_manufracturer_string(handle, wstr, MAX_STR);
    wprintf(L"Manufacturer String: %s\n", wstr);
    res = hid_get_product_string(handle, wstr, MAX_STR);
	wprintf(L"Product String: %s\n", wstr);
}


