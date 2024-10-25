/**
* @file
* @brief initialization file read and write API
* -size of the ini file must less than 16K
* -after '=' in key value pair, can not support empty char. this would not like WIN32 API
* -support comment using ';' prefix
* -can not support multi line
* @author Deng Yangjun
* @date 2012-1-9
* @version 1.2
*/

#ifndef READ_INI_FILE_H_
#define READ_INI_FILE_H_
int rf_readstringfromeprofile( const char *section, const char *key,char *value, int size,const char *default_value, const char *file);
int rf_readintfromeprofile( const char *section, const char *key,int default_value, const char *file);
int rf_writeintforprofile( const char *section, const char *key,const char *value, const char *file,const int array_num);
void rf_read2array( unsigned int ordArray[][3], char * read_buffer, unsigned int num);

#endif //end of INI_FILE_H_

