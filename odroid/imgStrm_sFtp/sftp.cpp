
#include "sftp.h"

#define HOST_IP "10.0.0.1"

using namespace std;

namespace sftp_namespace {

    int sftp_connect(ssh_session session, sftp_session &sftp)
    {
      int rc;  

      sftp = sftp_new(session);
      if (sftp == NULL)
      {
        fprintf(stderr, "Error allocating SFTP session: %s\n",
                ssh_get_error(session));
        return SSH_ERROR;
      }
      rc = sftp_init(sftp);
      if (rc != SSH_OK)
      {
        fprintf(stderr, "Error initializing SFTP session: %s.\n",
                sftp_get_error(sftp));
        sftp_free(sftp);
        return rc;
      }

       return SSH_OK;
    }

    int sftp_mkdir(ssh_session session, sftp_session sftp, string camId)
    {
      int rc;
      string dirName = "./images/" + camId;
      rc = sftp_mkdir(sftp, dirName.c_str(), S_IRWXU);
      if (rc != SSH_OK)
      {
        if (sftp_get_error(sftp) != SSH_FX_FILE_ALREADY_EXISTS)
        {
          fprintf(stderr, "Can't create directory: %s\n",
                  ssh_get_error(session));
            return rc;
        }
      }

      return SSH_OK;
    }

    int sftp_filetransfer(ssh_session session, sftp_session sftp, string camId, uint imgId, vector<uchar> buf)
    {
      int access_type = O_WRONLY | O_CREAT | O_TRUNC;
      sftp_file file;
      // const char *helloworld = "Hello, World!\n";
      int nwritten;
      int length = buf.size();
      int PACKET_SIZE =30*1024;
      int nWrites = floor(float(length) / PACKET_SIZE);
      int nRemains = length - nWrites * PACKET_SIZE;
      // cout << "length = " << length << endl;
      // cout << "nWrites = " << nWrites << endl;
      // cout << "nRemains = " << nRemains << endl;
      // Write the file remotely.
      stringstream ss;
      ss << setw(8) << setfill('0') << imgId;
      string filename = "./images/" + camId + "/" + ss.str() + ".jpg";
      file = sftp_open(sftp, filename.c_str(),
                       access_type, S_IRWXU);
      if (file == NULL)
      {
        fprintf(stderr, "Can't open file for writing: %s\n",
                ssh_get_error(session));
        return SSH_ERROR;
      }
      int i=0;
      for(; i<nWrites; i++)
      {  
          nwritten = sftp_write(file, buf.data() + i*PACKET_SIZE, PACKET_SIZE);
          if (nwritten != PACKET_SIZE)
          {
            fprintf(stderr, "Can't write data to file1: %s\n",
                    ssh_get_error(session));
            sftp_close(file);
            return SSH_ERROR;
          }
      }
      if(nRemains > 0)
      {
          nwritten = sftp_write(file, buf.data() + i*PACKET_SIZE, nRemains);
          if (nwritten != nRemains)
          {
            fprintf(stderr, "Can't write data to file2: %s\n",
                    ssh_get_error(session));
            sftp_close(file);
            return SSH_ERROR;
          }
      }

      int rc = sftp_close(file);
      if (rc != SSH_OK)
      {
        fprintf(stderr, "Can't close the written file: %s\n",
                ssh_get_error(session));
        return rc;
      }

      // sftp_free(sftp);
      return SSH_OK;
    }

    int verify_knownhost(ssh_session session)
    {
      int state, hlen;
      unsigned char *hash = NULL;
      char *hexa;
      char buf[10];
      state = ssh_is_server_known(session);
      hlen = ssh_get_pubkey_hash(session, &hash);
      if (hlen < 0)
        return -1;
      switch (state)
      {
        case SSH_SERVER_KNOWN_OK:
          break; /* ok */
        case SSH_SERVER_KNOWN_CHANGED:
          fprintf(stderr, "Host key for server changed: it is now:\n");
          ssh_print_hexa("Public key hash", hash, hlen);
          fprintf(stderr, "For security reasons, connection will be stopped\n");
          free(hash);
          return -1;
        case SSH_SERVER_FOUND_OTHER:
          fprintf(stderr, "The host key for this server was not found but an other"
            "type of key exists.\n");
          fprintf(stderr, "An attacker might change the default server key to"
            "confuse your client into thinking the key does not exist\n");
          free(hash);
          return -1;
        case SSH_SERVER_FILE_NOT_FOUND:
          fprintf(stderr, "Could not find known host file.\n");
          fprintf(stderr, "If you accept the host key here, the file will be"
           "automatically created.\n");
          /* fallback to SSH_SERVER_NOT_KNOWN behavior */
        case SSH_SERVER_NOT_KNOWN:
          hexa = ssh_get_hexa(hash, hlen);
          fprintf(stderr,"The server is unknown. Do you trust the host key?\n");
          fprintf(stderr, "Public key hash: %s\n", hexa);
          free(hexa);
          if (fgets(buf, sizeof(buf), stdin) == NULL)
          {
            free(hash);
            return -1;
          }
          if (strncasecmp(buf, "yes", 3) != 0)
          {
            free(hash);
            return -1;
          }
          if (ssh_write_knownhost(session) < 0)
          {
            fprintf(stderr, "Error %s\n", strerror(errno));
            free(hash);
            return -1;
          }
          break;
        case SSH_SERVER_ERROR:
          fprintf(stderr, "Error %s", ssh_get_error(session));
          free(hash);
          return -1;
      }
      free(hash);
      return 0;
    }

    int connectSsh(ssh_session &my_ssh_session)
    {
        int rc;
        char *password;
        my_ssh_session = ssh_new();
        if (my_ssh_session == NULL)
        {
            cout << "error in ssh_new\n";
            return(-1);
        }

        ssh_options_set(my_ssh_session, SSH_OPTIONS_HOST, HOST_IP);
        ssh_options_set(my_ssh_session, SSH_OPTIONS_USER, "mcsadmin");// _sftp

        // Connect to server
        rc = ssh_connect(my_ssh_session);
        if (rc != SSH_OK)
        {
            fprintf(stderr, "Error connecting to localhost: %s\n",
                    ssh_get_error(my_ssh_session));
            ssh_free(my_ssh_session);
            return(-1);
        }
        // Verify the server's identity
        // For the source code of verify_knowhost(), check previous example
        if (verify_knownhost(my_ssh_session) < 0)
        {
            ssh_disconnect(my_ssh_session);
            ssh_free(my_ssh_session);
            return(-1);
        }
        // Authenticate ourselves
        password = "temp1234";// ben getpass("Password: ");
        rc = ssh_userauth_password(my_ssh_session, NULL, password);
        if (rc != SSH_AUTH_SUCCESS)
        {
            fprintf(stderr, "Error authenticating with password: %s\n",
                    ssh_get_error(my_ssh_session));
            ssh_disconnect(my_ssh_session);
            ssh_free(my_ssh_session);
            return(-1);
        }
        
        return 0;
    }

}
