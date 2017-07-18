#ifndef BEN_SFTP_H

#define BEN_SFTP_H

#include "com.h"

using namespace std;


namespace sftp_namespace {

    int sftp_connect(ssh_session session, sftp_session &sftp);
    int sftp_mkdir(ssh_session session, sftp_session sftp, string camId);
    int sftp_filetransfer(ssh_session session, sftp_session sftp, string camId, uint imgId, vector<uchar> buf);
    int verify_knownhost(ssh_session session);
    int connectSsh(ssh_session &my_ssh_session);

}

#endif
