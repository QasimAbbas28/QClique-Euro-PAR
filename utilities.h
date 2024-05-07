#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <cstring>

using namespace std;

char * convertStringToChars(string str) {
    int n = str.length();

    static char * char_array = new char[n + 1];

    strcpy(char_array, str.c_str());

    return char_array;
}