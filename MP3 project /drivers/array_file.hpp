
#ifndef ARRAY_FILE_HPP_
#define ARRAY_FILE_HPP_

class array_file
{
public :
    TCHAR   fname[13];
    DWORD   file_size;
    TCHAR   sname[31] = {'\0'};
    TCHAR   aname[31] = {'\0'};
    static int count;
};



#endif /* ARRAY_FILE_HPP_ */
