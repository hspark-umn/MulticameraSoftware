#ifndef BEN_TCP_CLIENT_H
#define BEN_TCP_CLIENT_H

#include <string>
#include <vector>

using namespace std;

int connectServer(int &listenFd);
int tcp_filetransfer(int img_id, int &listenFd, string camId, vector<unsigned char> &buf);

#endif