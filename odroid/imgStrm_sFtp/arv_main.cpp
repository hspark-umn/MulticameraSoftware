#include "com.h"
#include "sftp.h"
#include <thread>
#include <mutex>

using namespace cv;
using namespace std;
using namespace std::chrono;
using namespace sftp_namespace;

#define MAX_STREAM_BUFFERS 20
#define MAX_BUFFERS 3

#define is_buf_writable(id_rd, id_wr) (((1 + id_wr - id_rd) % MAX_BUFFERS ) != 0) 
#define is_buf_readable(id_rd, id_wr) (id_rd != id_wr )

typedef struct {
	vector<uchar> img_jpg;
	int img_id;
} JpgData;


typedef struct {
	GMainLoop *main_loop;
	int buffer_count;
    mutex jpg_buf_mux;
    int buf_id;
    //vector<JpgData> imgbuf;
    string cam_id;
    int img_id;
    ssh_session my_ssh_session;
    sftp_session sftp;
} ApplicationData;


vector< vector<JpgData> > imgBuffer;
vector<int> id_BufRd, id_BufWr;
//vector<mutex> buf_mux;
mutex buf_mux[2];

static gboolean cancel = FALSE;
static int nCam = 0;
//static ssh_session my_ssh_session;
//static sftp_session sftp;


void sFTP_transfer(int buf_id, JpgData* imgbuf, string cam_id, ssh_session my_ssh_session, sftp_session sftp) // vector<JpgData>&
{
    while(!cancel)
    {
        {
            //unique_lock<mutex> lock(jpg_buf_mux);
            buf_mux[buf_id].lock();
        
            if(is_buf_readable(id_BufRd[buf_id], id_BufWr[buf_id]))
            {
                if(sftp_filetransfer(my_ssh_session, sftp, cam_id, imgbuf[id_BufRd[buf_id]].img_id, imgbuf[id_BufRd[buf_id]].img_jpg) != SSH_OK)
                {
                    cout << "File Sending Error\n";
                }
                id_BufRd[buf_id]++;
                if(id_BufRd[buf_id] >= MAX_BUFFERS)
                {
                    id_BufRd[buf_id] -= MAX_BUFFERS;
                }
            }
            buf_mux[buf_id].unlock(); 
        }    
          

    }
}


static void
set_cancel (int signal)
{
	cancel = TRUE;
}

static void
new_buffer_cb (ArvStream *stream, ApplicationData *data)
{
	ArvBuffer *buffer;

	buffer = arv_stream_try_pop_buffer (stream);
	if (buffer != NULL) {
		if (arv_buffer_get_status (buffer) == ARV_BUFFER_STATUS_SUCCESS) {
			data->buffer_count++;
		/* Image processing here */
            int width, height;
            char *buffer_data;
            size_t buffer_size;
            vector<uchar> img_buffer;
            

            buffer_data = (char *) arv_buffer_get_data (buffer, &buffer_size);
            arv_buffer_get_image_region (buffer, NULL, NULL, &width, &height);
            // arv_row_stride = width * ARV_PIXEL_FORMAT_BIT_PER_PIXEL (arv_buffer_get_image_pixel_format (buffer)) / 8;
            Mat image(height, width, CV_8UC1, buffer_data);// , gray_image; // , arv_row_stride                        
            imencode(".jpg", image, img_buffer);

            {
                //unique_lock<mutex> lock(data->jpg_buf_mux);
                buf_mux[data->buf_id].lock();
        
                if(is_buf_writable(id_BufRd[data->buf_id], id_BufWr[data->buf_id]))
                {
                    imgBuffer[data->buf_id][id_BufWr[data->buf_id]].img_id = data->img_id;
                    imgBuffer[data->buf_id][id_BufWr[data->buf_id]].img_jpg = img_buffer;
                    id_BufWr[data->buf_id]++;
                    if(id_BufWr[data->buf_id] >= MAX_BUFFERS)
                    {
                        id_BufWr[data->buf_id] -= MAX_BUFFERS;
                    }
                } 
                buf_mux[data->buf_id].unlock();
            }
            //if(sftp_filetransfer(data->my_ssh_session, data->sftp, data->cam_id, data->img_id, img_buffer) != SSH_OK)
            //{
            //    cout << "File Sending Error\n";
            //}
            data->img_id++;
        }
        else
        {
            cout << "buffer err\n";
        }


		arv_stream_push_buffer (stream, buffer);
	}
    else
    {
        cout << "null buffer read\n";
    }
}

static gboolean
periodic_task_cb (void *abstract_data)
{
	ApplicationData *data = (ApplicationData *)abstract_data; // (ApplicationData *)
    for(int i=0; i < nCam; i++)
    {
	    printf ("Rate(%d) = %d Hz\n", i, data[i].buffer_count);
	    data[i].buffer_count = 0;
    }

	if (cancel) {
		g_main_loop_quit (data[0].main_loop);
		return FALSE;
	}

	return TRUE;
}

static void
control_lost_cb (ArvGvDevice *gv_device)
{
	/* Control of the device is lost. Display a message and force application exit */
	printf ("Control lost\n");

	cancel = TRUE;
}

int main (int argc, char **argv)
{
    //ssh_session my_ssh_session;
    //sftp_session sftp;
    // string cam_id;

	/* Mandatory glib type system initialization */
	arv_g_type_init ();

    arv_update_device_list();
    nCam = arv_get_n_devices();
    cout << "Found " << nCam << " cameras.\n";
    if (nCam == 0) 
    {
        printf ("No Camera. Exiting...\n");
        return 0;
    }


    vector < string > sCamId(nCam);
    vector < ArvCamera* > camera(nCam);
    vector < ArvStream* > stream(nCam);
    vector < ApplicationData > data(nCam);
    vector < thread > vt;// (thread::hardware_concurrency()); // (nCam)
    //vector< vector<JpgData> > imgBuffer(nCam); 
    imgBuffer.resize(nCam);
    //buf_mux.resize(nCam);
    id_BufRd.resize(nCam, 0);
    id_BufWr.resize(nCam, 0);
    void (*old_sigint_handler)(int);
	int i;    

    for(i = 0; i < nCam; i++)
    {   
        imgBuffer[i].resize(MAX_BUFFERS);
        cout << "Testing ssh " << i << "\n";
        if(connectSsh(data[i].my_ssh_session) != 0)
        {
            return(-1);
        }

        if(sftp_connect(data[i].my_ssh_session, data[i].sftp) != SSH_OK)
        {
            for(int j=0; j <= i; j++)
            {
                sftp_free(data[j].sftp);
                ssh_disconnect(data[j].my_ssh_session);
                ssh_free(data[j].my_ssh_session);
            }
            return(-1);
        }
        cout << "SFTP " << i << " connected.\n";
	    data[i].buffer_count = 0;
        data[i].buf_id = i;
        data[i].img_id = 0;
        sCamId[i] = arv_get_device_id(i);
	    /* Instantiation of the first available camera */
	    camera[i] = arv_camera_new (sCamId[i].c_str()); // NULL

	    if (camera[i] == NULL) 
        {
	        printf ("Error in initializing cameras.\n");
            for(int j=0; j <= i; j++)
            {
                sftp_free(data[j].sftp);
                ssh_disconnect(data[j].my_ssh_session);
                ssh_free(data[j].my_ssh_session);
            }
	        return -1;
        }
        else
        {
		    
		    gint payload;
		    gint x, y, width, height;
		    gint dx, dy;
		    double exposure, rate;
		    guint64 n_completed_buffers;
		    guint64 n_failures;
		    guint64 n_underruns;
		    int gain;
		    guint software_trigger_source = 0;            

            data[i].cam_id = arv_camera_get_device_id(camera[i]);
            //cout << "device id = " << data[i].cam_id << endl;
            if(sftp_mkdir(data[i].my_ssh_session, data[i].sftp, data[i].cam_id) != SSH_OK)
            {
                for(int j=0; j <= i; j++)
                {
                    sftp_free(data[j].sftp);
                    ssh_disconnect(data[j].my_ssh_session);
                    ssh_free(data[j].my_ssh_session);
                }
                return(-2);
            }
//sFTP_transfer(uint buf_id, vector<JpgData>& imgbuf, string cam_id, ssh_session my_ssh_session, sftp_session sftp)
            vt.push_back(thread(sFTP_transfer, i, imgBuffer[i].data(), data[i].cam_id, data[i].my_ssh_session, data[i].sftp));
		    /* Set region of interrest to a 200x200 pixel area */
		    //arv_camera_set_region (camera, 0, 0, 200, 200);
		    /* Set frame rate to 10 Hz */
		    //arv_camera_set_frame_rate (camera[i], 25.0);
            //arv_camera_set_exposure_time (camera[i], 900); // us
		    arv_camera_get_region (camera[i], &x, &y, &width, &height);
		    arv_camera_get_binning (camera[i], &dx, &dy);
		    exposure = arv_camera_get_exposure_time (camera[i]);
            rate = arv_camera_get_frame_rate(camera[i]);
		    payload = arv_camera_get_payload (camera[i]);
		    gain = arv_camera_get_gain (camera[i]);
            cout << "************Camera " << i << endl;
		    printf ("vendor name           = %s\n", arv_camera_get_vendor_name (camera[i]));
		    printf ("model name            = %s\n", arv_camera_get_model_name (camera[i]));
		    printf ("device id             = %s\n", arv_camera_get_device_id (camera[i]));
		    printf ("image width           = %d\n", width);
		    printf ("image height          = %d\n", height);
		    printf ("horizontal binning    = %d\n", dx);
		    printf ("vertical binning      = %d\n", dy);
		    printf ("payload               = %d bytes\n", payload);
		    printf ("exposure              = %g µs\n", exposure);
            printf ("frame rate            = %g fps\n", rate);
            printf ("trigger source        = %s\n", arv_camera_get_trigger_source(camera[i]));
		    printf ("gain                  = %d dB\n", gain);


		    /* Create a new stream object */
		    stream[i] = arv_camera_create_stream (camera[i], NULL, NULL);
		    if (stream[i] == NULL) 
            {
                printf ("Can't create stream thread (check if the device is not already used)\n");
                for(int j=0; j <= i; j++)
                {
                    sftp_free(data[j].sftp);
                    ssh_disconnect(data[j].my_ssh_session);
                    ssh_free(data[j].my_ssh_session);
                }
	            return -3;
            }
            else
            {
			    /* Push 50 buffer in the stream input buffer queue */
			    for (int j = 0; j < MAX_STREAM_BUFFERS; j++)
				    arv_stream_push_buffer (stream[i], arv_buffer_new (payload, NULL));
            }
        }
    }

    for(i = 0; i < nCam; i++)
    {   

	    ///* Start the video stream */
	    // arv_camera_start_acquisition (camera[i]);

	    /* Connect the new-buffer signal */
	    g_signal_connect (stream[i], "new-buffer", G_CALLBACK (new_buffer_cb), &data[i]);
	    /* And enable emission of this signal (it's disabled by default for performance reason) */
	    arv_stream_set_emit_signals (stream[i], TRUE);

	    /* Connect the control-lost signal */
	    g_signal_connect (arv_camera_get_device (camera[i]), "control-lost",
			      G_CALLBACK (control_lost_cb), NULL);
    }


	/* Install the callback for frame rate display */
	g_timeout_add_seconds (1, periodic_task_cb, data.data());

	/* Create a new glib main loop */
	data[0].main_loop = g_main_loop_new (NULL, FALSE);

	old_sigint_handler = signal (SIGINT, set_cancel);

    for(i = nCam-1; i >= 0; i--)
    { 
    	/* Start the video stream */
	    arv_camera_start_acquisition (camera[i]);
    }

	/* Run the main loop */
	g_main_loop_run (data[0].main_loop);

	signal (SIGINT, old_sigint_handler);

	g_main_loop_unref (data[0].main_loop);

    for(i = 0; i < nCam; i++)
    {   

	    /* Stop the video stream */
	    arv_camera_stop_acquisition (camera[i]);

	    /* Signal must be inhibited to avoid stream thread running after the last unref */
	    arv_stream_set_emit_signals (stream[i], FALSE);
        vt[i].join();

	    g_object_unref (stream[i]);
        g_object_unref (camera[i]);
        sftp_free(data[i].sftp);
        ssh_disconnect(data[i].my_ssh_session);
        ssh_free(data[i].my_ssh_session);
    }




	return 0;
}
