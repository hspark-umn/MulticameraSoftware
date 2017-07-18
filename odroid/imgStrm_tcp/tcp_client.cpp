#include <string.h>
#include <cstring>
#include <unistd.h>
#include <stdio.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <strings.h>
#include <stdlib.h>
#include <string>
#include <time.h>
#include <vector>
using namespace std;

// int listenFd;
// extern vector<int> nFrameIds;

int connectServer(int &listenFd) //main (int argc, char* argv[])
{
    int portNo;//listenFd, 
    bool loop = false;
    struct sockaddr_in svrAdd;
    struct hostent *server;
    
    // if(argc < 3)
    // {
    //     cerr<<"Syntax : ./client <host name> <port>"<<endl;
    //     return 0;
    // }
    
    portNo = 8888;//atoi(argv[2]);//
    
    // if((portNo > 65535) || (portNo < 2000))
    // {
    //     cerr<<"Please enter port number between 2000 - 65535"<<endl;
    //     return 0;
    // }       
    
    //create client skt
    listenFd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    
    if(listenFd < 0)
    {
        cerr << "Cannot open socket" << endl;
        return -1;
    }
    
    // server = gethostbyname("multicam2-umh");//argv[1]
    
    // if(server == NULL)
    // {
    //     cerr << "Host does not exist" << endl;
    //     return -2;
    // }
    
    bzero((char *) &svrAdd, sizeof(svrAdd));
    svrAdd.sin_family = AF_INET;

    string host_ip("10.0.0.1");
    // bcopy(host_ip.c_str(), (char *) &svrAdd.sin_addr.s_addr, host_ip.length());
    inet_aton(host_ip.c_str(), (in_addr *)(&svrAdd.sin_addr.s_addr));
    // bcopy(h_addr, (char *) &svrAdd.sin_addr.s_addr, strlen(h_addr));
    
    svrAdd.sin_port = htons(portNo);
    
    int checker = connect(listenFd,(struct sockaddr *) &svrAdd, sizeof(svrAdd));
    
    if (checker < 0)
    {
        cerr << "Cannot connect!" << endl;
        return -3;
    }
    
    //send stuff to server
    // for(;;)
    // {
    //     char s[300];
    //     //cin.clear();
    //     //cin.ignore(256, '\n');
    //     cout << "Enter stuff: ";
    //     bzero(s, 300);
    //     cin.getline(s, 300);
        
    //     write(listenFd, s, strlen(s));
    // }
    return 0;
}

// int sendTcpData(char * s)
// {
//     write(listenFd, s, strlen(s));
// }

#define CAMID_LEN 8
#define FRAMEID_LEN 10
#define FILESIZE_LEN 6
#define HEAD_LEN (CAMID_LEN + FRAMEID_LEN + FILESIZE_LEN)

int tcp_filetransfer(int img_id, int &listenFd, string camId, vector<unsigned char> &buf)
{
  int nWr, nRet, nToSend;
  int nLen = int(buf.size());
  int PACKET_SIZE = 600;//1200; //100

  // int nWrites = floor(float(nLen) / PACKET_SIZE);
  // int nRemains = nLen - nWrites * PACKET_SIZE;
  // cout << "fs=" << nLen << endl;

  stringstream ss_flen, ss_fid;
  ss_flen << setw(FILESIZE_LEN) << setfill('0') << nLen;
  ss_fid << setw(FRAMEID_LEN) << setfill('0') << img_id;
  string head = string("***") + camId + ss_fid.str() + ss_flen.str();// 

  // Send header
  nWr = 0;
  while(nWr < head.length())
  {
    nRet = write(listenFd, head.c_str() + nWr, head.length() - nWr);
    if(nRet < 0) 
    {
      cerr << "ERROR writing to socket (H)\n";
      return(-1);
    }
    nWr += nRet;
  }

  // Send data body
  nWr = 0;

  while(nWr < nLen)
  {  
      nToSend = nLen - nWr;
      if(nToSend > PACKET_SIZE)
      {
        nToSend = PACKET_SIZE;
      }
      nRet = write(listenFd, buf.data() + nWr, nToSend);
      if(nRet < 0) 
      {
        cerr << "ERROR writing to socket (B)\n";
        return(-2);
      }
      nWr += nRet;
  }

// cout << "cid=" << camId << "pid=" << img_id << " sent.\n";

  return 0;
}
