/**
* @file
* @brief initialization file read and write API implementation
* @author Deng Yangjun
* @date 2012-1-9
* @version 1.2
*/
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include "readinifile.h"
#include "tpthread.h"
#include "netbuss.h"


#define LEFT_BRACE '['
#define RIGHT_BRACE ']'

static int load_ini_file(const char *file, char *buf,int *file_size)
{
	FILE *in = NULL;
	int i=0;
	*file_size =0;
	assert(file !=NULL);
	assert(buf !=NULL);
	in = fopen(file,"r");
	if( NULL == in) {
		return 0;
	}
	buf[i]=fgetc(in);
	//load initialization file
	while( buf[i]!= (char)EOF) {
		i++;
		assert( i < MAX_FILE_SIZE ); //file too big, you can redefine 	MAX_FILE_SIZE to fit the big file
		buf[i] = fgetc(in);
	}
	buf[i] = '\0';
	*file_size = i;
	fclose(in);
	return 1;
}

static int newline(char c)
{
	return ('\n' == c || '\r' == c )? 1 : 0;
}

static int end_of_string(char c)  // ��0
{
	return '\0'==c ? 1 : 0;
//	return('|' == c) || ('\0'== c)? 1 : 0;
}

static int left_barce(char c)
{
	return LEFT_BRACE == c ? 1 : 0;
}

static int isright_brace(char c )
{
	return RIGHT_BRACE == c ? 1 : 0;
}


/*   sec_s:  section start  sec_e: section_end   key_s: key start   key_e: key_end   */
static int parse_file(const char *section, const char *key, const char *buf, int *sec_s, int *sec_e, int *key_s, \
				int *key_e, int *value_s, int *value_e)
{
	const char *p = buf;
	int i=0;
	assert(buf!=NULL);
	assert(section != NULL && strlen(section));
	assert(key != NULL && strlen(key));
	*sec_e = *sec_s = *key_e = *key_s = *value_s = *value_e = -1;
	while( !end_of_string(p[i]) ) {
		//find the section
		if( ( 0==i || newline(p[i-1]) ) && left_barce(p[i]) )
		{
			int section_start = i+1;
			//find the ']'
			do {
				i++;
			} while( !isright_brace(p[i]) && !end_of_string(p[i]));
			
			if( 0 == strncmp(p + section_start, section, i - section_start)) {
				int newline_start = 0;
				i++;
				//Skip over space char after ']'
				while(isspace(p[i])) {
					i++;
				}
				
				//find the section
				*sec_s = section_start;
				*sec_e = i;
				while( ! (newline(p[i-1]) && left_barce(p[i]))
					&& !end_of_string(p[i]) ) {
					int j=0;
					//get a new line
					newline_start = i;
					while( !newline(p[i]) && !end_of_string(p[i]) ) {
						i++;
					}
						//now i is equal to end of the line
					j = newline_start;
					if(';' != p[j]) //skip over comment
					{
						while(j < i && p[j]!='=') {
							j++;
							if('=' == p[j]) {
								//printf("[parse_file]: key = %s, %c%c, %d, j = %d, i = %d\n\n", key, *(p+newline_start), *(p+newline_start+1), 
								//	j-newline_start, j, i);
								if((strncmp(key,p+newline_start,j-newline_start)==0) 
								&& ( (j - newline_start) == strlen(key) ))
								{
									//find the key ok
									*key_s = newline_start;
									*key_e = j-1;
									*value_s = j+1;
									
									*value_e = i;
									return 1;
								}
							}
						}
					}
					i++;
				}
			}
		}
		else
		{
			i++;
		}
	}
	return 0;
}

/**	read string in initialization file\n
* retrieves a string from the specified section in an initialization file
   section [in] name of the section containing the key name
   key [in] name of the key pairs to value
   value [in] pointer to the buffer that receives the retrieved string
   size [in] size of result's buffer
   default_value [in] default value of result
   file [in] path of the initialization file
1 : read success; \n 0 : read fail
*/
int rf_readstringfromeprofile( const char *section, const char *key, char *value, int size, const char *default_value, const char *file)
{
	char buf[MAX_FILE_SIZE]={0};
	int file_size;
	int sec_s,sec_e,key_s,key_e, value_s, value_e;

	//check parameters
	assert(section != NULL && strlen(section));
	assert(key != NULL && strlen(key));
	assert(value != NULL);
	assert(size > 0);
	assert(file !=NULL &&strlen(key));
	
	if(!load_ini_file(file, buf, &file_size))
	{
		if(default_value != NULL)
		{
			strncpy(value, default_value, size);
		}
		return 0;
	}
	if(!parse_file(section, key, buf, &sec_s, &sec_e, &key_s, &key_e, &value_s, &value_e))
	{
		if(default_value != NULL)
		{
			strncpy(value, default_value, size);
		}
		return 0; //not find the key
	}
	else
	{
		int cpcount = value_e - value_s;   //  value_e :  value end.  value_s : value start
		if( size - 1 < cpcount)
		{
			cpcount = size - 1;
		}
		memset(value, 0, size);
		memcpy(value, buf + value_s, cpcount );
		value[cpcount] = '\0';
		return 1;
	}
}


/**read int value in initialization file\n
* retrieves int value from the specified section in an initialization file section [in] name of the section containing the key name
key [in] name of the key pairs to value
default_value [in] default value of result
file [in] path of the initialization file
profile int value,if read fail, return default value
*/
int rf_readintfromeprofile( const char *section, const char *key, int default_value, const char *file)
{
	char value[32] = {0};
	if(!rf_readstringfromeprofile(section, key, value, sizeof(value), NULL, file))
	{
		// exit(NULL);
		return default_value;
	}
	else
	{
		return atoi(value);
	}
}

int change_array_val(const char* buf_s,int * buf_len, const int array_num, const char* value, int value_len)
{
	int array_cnt = 0,i = 0,value_e = 0, value_s = 0;
	char buf_e[1000] = {0};

	for (array_cnt = 0; array_cnt < array_num;array_cnt++,i++)
	{
		while (buf_s[i] != '|' && buf_s[i] != 0)
			i++;
	}
		
	value_s = i;// find the start of value
	while (buf_s[i] != '|' && buf_s[i] != 0)
		i++;
	value_e = i;	//find the end of value

	printf("*buf_len=%d value_s = %d value_e =%d, array_num=%d\n", *buf_len, value_s, value_e, array_num);
	printf("value = %s value_len=%d\n", value, value_len);

	memcpy(buf_e, &buf_s[value_e], *buf_len - value_e);

	printf("buf_e = %s\n",buf_e);

	memcpy((char*)(&buf_s[value_s]), value, value_len);
	memcpy((char*)(&buf_s[value_s + value_len]), buf_e, *buf_len - value_e);
	
	*buf_len += value_len - (value_e - value_s);

	printf("*buf_len = %d\n", *buf_len);
	return 0;
}

/**
* @brief write a profile string to a ini file
* @param section [in] name of the section,can't be NULL and empty string
* @param key [in] name of the key pairs to value, can't be NULL and empty string
* @param value [in] profile string value
* @param file [in] path of ini file
* @param array_num [in] num of the '|' ,if the only one param the array_num must be 0
* @return 1 : success\n 0 : failure
*/
int rf_writeintforprofile(const char *section, const char *key, const char *
	value, const char *file, const int array_num)
{ 
	char buf[MAX_FILE_SIZE]={0};
	char w_buf[MAX_FILE_SIZE]={0};
	char str[1000] = {0};
	int sec_s,sec_e,key_s,key_e, value_s, value_e = 0;
	int value_len = (int)strlen(value), str_len = 0;
	int file_size;
	FILE *out = NULL;
	
	//check parameters
	assert(section != NULL && strlen(section));
	assert(key != NULL && strlen(key));
	assert(value != NULL);
	assert(file !=NULL &&strlen(key));
	if(!load_ini_file(file,buf,&file_size))
	{
		sec_s = -1;
	}
	else
	{
		parse_file(section,key,buf,&sec_s,&sec_e,&key_s,&key_e,&value_s,&value_e);
	}
	if( -1 == sec_s)
	{
		if(0==file_size)
		{
			sprintf(w_buf+file_size,"[%s]\n%s=%s\n",section,key,value);
		}
		else
		{
			//not find the section, then add the new section at end of the file
			memcpy(w_buf,buf,file_size);
			sprintf(w_buf+file_size,"\n[%s]\n%s=%s\n",section,key,value);
		}
	}
	else if(-1 == key_s)
	{
		//not find the key, then add the new key=value at end of the section
		memcpy(w_buf,buf,sec_e);
		sprintf(w_buf+sec_e,"%s=%s\n",key,value);
		sprintf(w_buf+sec_e+strlen(key)+strlen(value)+2,buf+sec_e, file_size - sec_e);
	}
	else
	{
		//update value with new value
		printf("update value with new value\n");

		//if (array_num > 0 && buf[value_e] != '\0')
		
		str_len = value_e - value_s;
		memcpy(str, buf+value_s, str_len);
				printf("str = %s str_len = %d\n",str, str_len);
		change_array_val(str, &str_len, array_num, value, value_len);

		memcpy(w_buf,buf,value_s);
		memcpy(w_buf+value_s,str, str_len);
		memcpy(w_buf+value_s+str_len, buf+value_e, file_size - value_e);		
	}
	
	out = fopen(file,"w");
	
	if(NULL == out)
	{
		printf("file not find\n");
		return 0;
	}
	if(-1 == fputs(w_buf,out) )
	{
		printf("file write error\n");
		fclose(out);
		return 0;
	}
	
	fclose(out);
	return 1;
}



/******************************************************************************
*
* Function Name  : rf_read2minidata
* Description    : .���ַ�����ȡ����ֵ һά����
* 					 
* Input		   :char * �ַ�����unsigned int �������
* Output		   :unsigned int ordArray[]��ȡ���ֵ��������
* Return		   :  None
*******************************************************************************/
void rf_read2minidata( unsigned int ordArray[], char* read_buffer, unsigned int num)
{
	// char pstring[1000] = {0};
	char *pstr = malloc(1000);
	if(pstr == NULL){
		lprintf(log_my, ERROR, "read mini parameet mallo fault.\n");
		return;
	}
	unsigned int x = 0, i;
	char *ptr = NULL;
	for (i = 0; i < num; i++)
	{
		strncpy(pstr, read_buffer, strlen(read_buffer));
		x = ordArray[i] = atoi(strtok_r(pstr, "|", &ptr));
		
		strncpy(pstr, read_buffer, strlen(read_buffer));
		strcpy(read_buffer, &read_buffer[strlen(strtok_r(pstr, "|", &ptr)) + 1]);
	}
}

/******************************************************************************
*
* Function Name  : rf_read2array
* Description    : .���ַ�����ȡ����ֵ��ά����
* 					 
* Input		   :char * �ַ�����unsigned int �������
* Output		   :unsigned int ordArray[][2]��ȡ���ֵ��������
* Return		   :  None
*******************************************************************************/
void rf_read2array( unsigned int ordArray[][3], char * read_buffer, unsigned int num)
{
	// char pstring[100] = {0};//��ȡ " "
	// char ppstring[100] = {0};//��ȡ "|"
	char *pstring = malloc(100);
	char *ppstring = malloc(100);
	unsigned int x = 0, y = 0, z = 0, i = 0, len = 0;
	char *ptr = NULL;

	printf("Into rf_read2array befor strtok_r.\n");

	for (i = 0; i < num; i++)
	{
		strcpy(ppstring, read_buffer);
		strtok_r(ppstring, "|", &ptr);
		len = strlen(ppstring);
		strcpy(pstring, ppstring);
		x = (ordArray[i][0]) = atoi(strtok_r(pstring, " ", &ptr));

		strcpy(ppstring, &ppstring[strlen(pstring) + 1]);
		strcpy(pstring, ppstring);	
		y = ordArray[i][1] = atoi(strtok_r(pstring, " ", &ptr));

		strcpy(ppstring, &ppstring[strlen(pstring) + 1]);
		strcpy(pstring, ppstring);	
		z = ordArray[i][2] = atoi(strtok_r(pstring, "|", &ptr));
		strcpy(read_buffer,&read_buffer[len + 1]);
	}

	free(pstring);
	free(ppstring);
}



/**************************���³�ʼ���ļ�*******************************/
void mb_updateconfilefrornet(void)
{
	unsigned int i = 0;
	int netread_len;
	FILE * fp=NULL;
	
	remove(MeasureConfigPath);
	if ((fp = fopen(MeasureConfigPath, "w+")) == NULL)
		perror("open file error\n");
	while(!flg_mainproexit)						//��ȡ��������
	{	
		memset(netbuf_read, 0, 1000);
		while(1)
		{
			if ((netread_len = read(local_fd, netbuf_read, 1000)) < 0)//Ҫ��pc��һ����������1000�ֽڣ�һ��Ϊһ�Ų�Ƭ�Ĳ���
			{
			//	perror("[mb_updateconfilefrornet]read netdata error\n");
			}
			else 
				break;
			usleep(1000);
		}

		printf("[mb_updateconfilefrornet]netdata recieved\n");	//������Ϣ��ʹ���ַ�����ʽ�շ�			
		printf("netread_len=%d netbuf_read=  \n", netread_len); 
		printf("\n");
		
		//	printf("fgetc(fp)=%c\n",fgetc(fp));
		for (i = 0; i < netread_len; i++)
		{
			printf("%c", netbuf_read[i]);
			if (fputc(netbuf_read[i], fp) < 0)
				perror("fput error\n");
		}

		if (strcmp((char*)&netbuf_read[netread_len - 3], "END") != 0)	//end of the file
			continue;
		else
		{
			fclose(fp);
			break;
		}
		printf("\n");

		usleep(1000);	
	}
}


