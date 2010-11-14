#include <linux/proc_fs.h>
#include <linux/module.h>
#include <asm/gpio.h>
#include <mach/msm_iomap.h>
#include <mach/msm_smd.h>

/* FIH, AudiPCHuang, 2009/06/05 { */
/* [ADQ.B-1440], For getting image version from splash.img*/
static char adq_image_version[32];
int JogballExist_pr1,JogballExist_pr2;

#define PLUS_X_GPIO		91
#define NEG_X_GPIO   	88
#define PLUS_Y_GPIO  	90
#define NEG_Y_GPIO   	93

void set_image_version(char* img_ver)
{
	snprintf(adq_image_version, 25, "%s", img_ver);
}
EXPORT_SYMBOL(set_image_version);
/* } FIH, AudiPCHuang, 2009/06/05 */

static int proc_calc_metrics(char *page, char **start, off_t off,
				 int count, int *eof, int len)
{
	if (len <= off+count) *eof = 1;
	*start = page + off;
	len -= off;
	if (len>count) len = count;
	if (len<0) len = 0;
	return len;
}

static int build_version_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len;

	len = snprintf(page, PAGE_SIZE, "%s\n", /*ADQ_IMAGE_VERSION*/adq_image_version);
	return proc_calc_metrics(page, start, off, count, eof, len);
}

static int device_model_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len;
	//int HWID = FIH_READ_HWID_FROM_SMEM();
	int HWID = FIH_READ_ORIG_HWID_FROM_SMEM();
	char ver[24];
		
	switch (HWID){
	case CMCS_HW_VER_EVB1:
		strcpy(ver, "F02");
		break; 
	case CMCS_HW_VER_EVB2:
		strcpy(ver, "F02");
		break; 
	case CMCS_HW_VER_EVB3:
		strcpy(ver, "F02");
		break; 
	case CMCS_ORIG_RTP_PR1:
		if (JogballExist_pr1)
			strcpy(ver, "F10");
		else
			strcpy(ver, "F02");
		break; 
	case CMCS_850_RTP_PR2:
		if (JogballExist_pr2)
			strcpy(ver, "F10");
		else
			strcpy(ver, "F02");
		break;    
	case CMCS_850_RTP_PR3:
		if (JogballExist_pr2)
			strcpy(ver, "F10");
		else
			strcpy(ver, "F02");
		break; 
	case CMCS_850_RTP_MP1:
		if (JogballExist_pr2)
			strcpy(ver, "F10");
		else
			strcpy(ver, "F02");
		break;    
	case CMCS_850_RTP_MP2:
		if (JogballExist_pr2)
			strcpy(ver, "F10");
		else
			strcpy(ver, "F02");
		break; 
	case CMCS_850_RTP_MP3:
		if (JogballExist_pr2)
			strcpy(ver, "F10");
		else
			strcpy(ver, "F02");
		break; 
	case CMCS_ORIG_CTP_PR1:
		if (JogballExist_pr1)
			strcpy(ver, "F11");
		else
			strcpy(ver, "F03");
		break; 
	case CMCS_850_CTP_PR2:
		if (JogballExist_pr2)
			strcpy(ver, "F11");
		else
			strcpy(ver, "F03");
		break;    
	case CMCS_850_CTP_PR3:
		if (JogballExist_pr2)
			strcpy(ver, "F11");
		else
			strcpy(ver, "F03");
		break; 
	case CMCS_850_CTP_MP1:
		if (JogballExist_pr2)
			strcpy(ver, "F11");
		else
			strcpy(ver, "F03");
		break;    
	case CMCS_850_CTP_MP2:
		if (JogballExist_pr2)
			strcpy(ver, "F11");
		else
			strcpy(ver, "F03");
		break; 
	case CMCS_850_CTP_MP3:
		if (JogballExist_pr2)
			strcpy(ver, "F11");
		else
			strcpy(ver, "F03");
		break; 
	case CMCS_900_RTP_PR2:
		if (JogballExist_pr2)
			strcpy(ver, "F10");
		else
			strcpy(ver, "F02");
		break;    
	case CMCS_900_RTP_PR3:
		if (JogballExist_pr2)
			strcpy(ver, "F10");
		else
			strcpy(ver, "F02");
		break; 
	case CMCS_900_RTP_MP1:
		if (JogballExist_pr2)
			strcpy(ver, "F10");
		else
			strcpy(ver, "F02");
		break;    
	case CMCS_900_RTP_MP2:
		if (JogballExist_pr2)
			strcpy(ver, "F10");
		else
			strcpy(ver, "F02");
		break; 
	case CMCS_900_RTP_MP3:
		if (JogballExist_pr2)
			strcpy(ver, "F10");
		else
			strcpy(ver, "F02");
		break; 
	case CMCS_900_CTP_PR2:
		if (JogballExist_pr2)
			strcpy(ver, "F11");
		else
			strcpy(ver, "F03");
		break;    
	case CMCS_900_CTP_PR3:
		if (JogballExist_pr2)
			strcpy(ver, "F11");
		else
			strcpy(ver, "F03");
		break; 
	case CMCS_900_CTP_MP1:
		if (JogballExist_pr2)
			strcpy(ver, "F11");
		else
			strcpy(ver, "F03");
		break;    
	case CMCS_900_CTP_MP2:
		if (JogballExist_pr2)
			strcpy(ver, "F11");
		else
			strcpy(ver, "F03");
		break; 
	case CMCS_900_CTP_MP3:
		if (JogballExist_pr2)
			strcpy(ver, "F11");
		else
			strcpy(ver, "F03");
		break; 
	case CMCS_145_CTP_PR1:
		strcpy(ver, "AWS");
		break; 
	case CMCS_125_FST_PR1:
		strcpy(ver, "FST");
		break; 
	case CMCS_125_FST_PR2:
		strcpy(ver, "FST");
		break;    
	case CMCS_125_FST_MP1:
		strcpy(ver, "FST");
		break; 
	case CMCS_128_FST_PR1:
		strcpy(ver, "FST");
		break; 
	case CMCS_128_FST_PR2:
		strcpy(ver, "FST");
		break;    
	case CMCS_128_FST_MP1:
		strcpy(ver, "FST");
		break; 
	case CMCS_125_CTP_GRE_PR1:
	case CMCS_125_CTP_GRE_PR2:
	case CMCS_125_CTP_GRE_MP1:
	case CMCS_125_CTP_GRE_MP2:
		strcpy(ver, "GRE");
		break;
	/* FIH, Debbie, 2010/05/04 { */
	case CMCS_125_FA9_PR1:
	case CMCS_125_FA9_PR2:
	case CMCS_125_FA9_PR3:
	case CMCS_125_FA9_MP1:
		strcpy(ver, "F19");
		break;
	/* FIH, Debbie, 2010/05/04 } */
	case CMCS_7627_ORIG_EVB1:
		strcpy(ver, "F13");
		break;    
	case CMCS_7627_F905_PR1:
	case CMCS_7627_F905_PR2:
	case CMCS_7627_F905_PR3:
		strcpy(ver, "F05");
		break; 
	case CMCS_7627_F913_PR1:
	case CMCS_7627_F913_PR2:
	case CMCS_7627_F913_PR3:		
	case CMCS_7627_F913_MP1_W: 
	case CMCS_7627_F913_MP1_C_G:
	/* FIH, Debbie, 2010/05/24 { */
	case CMCS_7627_F913_MP1_W_4G4G:
	case CMCS_7627_F913_MP1_C_G_4G4G:
	/* FIH, Debbie, 2010/05/24 } */
		strcpy(ver, "F13");
		break;
	/* FIH, Debbie, 2010/05/04 { */
	case CMCS_CTP_F917_PR1:	
	case CMCS_CTP_F917_PR2:	
	case CMCS_CTP_F917_PR3:		
	case CMCS_CTP_F917_PR4:	
	case CMCS_CTP_F917_PR5:
	case CMCS_CTP_F917_MP1:	
	case CMCS_CTP_F917_MP2:	
	case CMCS_CTP_F917_MP3:
		strcpy(ver, "F17");
		break;

	/* FIH, Debbie, 2010/05/24 { */
	case CMCS_125_4G4G_FAA_PR1:
	case CMCS_125_4G4G_FAA_PR2:
	case CMCS_125_4G4G_FAA_PR3:
	case CMCS_125_4G4G_FAA_MP1:
	case CMCS_128_4G4G_FAA_PR1:
	case CMCS_128_4G4G_FAA_PR2:
	case CMCS_128_4G4G_FAA_PR3:
	case CMCS_128_4G4G_FAA_MP1:
		strcpy(ver, "FAA");
		break; 	
        /* FIH, Debbie, 2010/05/24 } */
	
	case CMCS_7627_F20_PR1:
	case CMCS_7627_F20_PR2:
	case CMCS_7627_F20_PR3:
	case CMCS_7627_F20_MP1:
		strcpy(ver, "F20");
		break;
	/* FIH, Debbie, 2010/05/04 } */	
	default:
		strcpy(ver, "Unkonwn Device Model");
		break;
	}

	len = snprintf(page, PAGE_SIZE, "%s\n",
		ver);
		
	return proc_calc_metrics(page, start, off, count, eof, len);	
}

static int baseband_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len;

	//int HWID = FIH_READ_HWID_FROM_SMEM();
	int HWID = FIH_READ_ORIG_HWID_FROM_SMEM();
	char ver[24];
	
	switch (HWID){
	case CMCS_HW_VER_EVB1:
		strcpy(ver, "EVB1");
		break; 
	case CMCS_HW_VER_EVB2:
		strcpy(ver, "EVB2");
		break; 
	case CMCS_HW_VER_EVB3:
		strcpy(ver, "EVB3");
		break; 
	case CMCS_ORIG_RTP_PR1:
		strcpy(ver, "PR1");
		break; 
	case CMCS_850_RTP_PR2:
		strcpy(ver, "PR2_850");
		break;    
	case CMCS_850_RTP_PR3:
		strcpy(ver, "PR3_850");
		break; 
	case CMCS_850_RTP_MP1:
		strcpy(ver, "MP1_850");
		break;    
	case CMCS_850_RTP_MP2:
		strcpy(ver, "MP2_850");
		break; 
	case CMCS_850_RTP_MP3:
		strcpy(ver, "MP3_850");
		break; 
	case CMCS_ORIG_CTP_PR1:
		strcpy(ver, "PR1");
		break; 
	case CMCS_850_CTP_PR2:
		strcpy(ver, "PR2_850");
		break;    
	case CMCS_850_CTP_PR3:
		strcpy(ver, "PR3_850");
		break; 
	case CMCS_850_CTP_MP1:
		strcpy(ver, "MP1_850");
		break;    
	case CMCS_850_CTP_MP2:
		strcpy(ver, "MP2_850");
		break; 
	case CMCS_850_CTP_MP3:
		strcpy(ver, "MP3_850");
		break; 
	case CMCS_900_RTP_PR2:
		strcpy(ver, "PR2_900");
		break;    
	case CMCS_900_RTP_PR3:
		strcpy(ver, "PR3_900");
		break; 
	case CMCS_900_RTP_MP1:
		strcpy(ver, "MP1_900");
		break;    
	case CMCS_900_RTP_MP2:
		strcpy(ver, "MP2_900");
		break; 
	case CMCS_900_RTP_MP3:
		strcpy(ver, "MP3_900");
		break; 
	case CMCS_900_CTP_PR2:
		strcpy(ver, "PR2_900");
		break;    
	case CMCS_900_CTP_PR3:
		strcpy(ver, "PR3_900");
		break; 
	case CMCS_900_CTP_MP1:
		strcpy(ver, "MP1_900");
		break;    
	case CMCS_900_CTP_MP2:
		strcpy(ver, "MP2_900");
		break; 
	case CMCS_900_CTP_MP3:
		strcpy(ver, "MP3_900");
		break; 
	case CMCS_145_CTP_PR1:
		strcpy(ver, "PR1_850");
		break; 
	case CMCS_125_FST_PR1:
		strcpy(ver, "PR1_850");
		break; 
	case CMCS_125_FST_PR2:
		strcpy(ver, "PR2_850");
		break;    
	case CMCS_125_FST_MP1:
		strcpy(ver, "MP1_850");
		break; 
	case CMCS_128_FST_PR1:
		strcpy(ver, "PR1_900");
		break; 
	case CMCS_128_FST_PR2:
		strcpy(ver, "PR2_900");
		break;    
	case CMCS_128_FST_MP1:
		strcpy(ver, "MP1_900");
		break; 
	case CMCS_125_CTP_GRE_PR1:
		strcpy(ver, "PR1");
		break;
	case CMCS_125_CTP_GRE_PR2:
		strcpy(ver, "PR2");
		break;
	case CMCS_125_CTP_GRE_MP1:
		strcpy(ver, "MP1");
		break;
	case CMCS_125_CTP_GRE_MP2:
		strcpy(ver, "MP2");
		break;
	/* FIH, Debbie, 2010/05/04 { */
	case CMCS_125_FA9_PR1:
		strcpy(ver, "PR1");
		break;
	case CMCS_125_FA9_PR2:
		strcpy(ver, "PR2");
		break;
	case CMCS_125_FA9_PR3:
		strcpy(ver, "PR3");
		break;
	case CMCS_125_FA9_MP1:
		strcpy(ver, "MP1");
		break;
	/* FIH, Debbie, 2010/05/04 } */
	case CMCS_7627_ORIG_EVB1:
		strcpy(ver, "EVB1");
		break;    
	case CMCS_7627_F905_PR1:
		strcpy(ver, "PR1");
		break; 
	case CMCS_7627_F905_PR2:
		strcpy(ver, "PR2");
		break; 
	case CMCS_7627_F905_PR3:
		strcpy(ver, "PR3");
		break; 
	case CMCS_7627_F913_PR1:
		strcpy(ver, "PR1");
		break; 
	case CMCS_7627_F913_PR2:
		strcpy(ver, "PR2");
		break; 
	case CMCS_7627_F913_PR3:
		strcpy(ver, "PR3");
		break; 		
	case CMCS_7627_F913_MP1_W:
	case CMCS_7627_F913_MP1_W_4G4G:  //FIH, Debbie, 2010/05/24
		strcpy(ver, "MP1_W");
		break; 
	case CMCS_7627_F913_MP1_C_G:
	case CMCS_7627_F913_MP1_C_G_4G4G:  //FIH, Debbie, 2010/05/24
		strcpy(ver, "MP1_C_G");
		break; 		
	case CMCS_CTP_F917_PR1:
		strcpy(ver, "PR1");
		break; 		
	case CMCS_CTP_F917_PR2:
		strcpy(ver, "PR2");
		break; 		
	case CMCS_CTP_F917_PR3:
		strcpy(ver, "PR3");
		break; 		
	case CMCS_CTP_F917_PR4:
		strcpy(ver, "PR4");
		break; 		
	case CMCS_CTP_F917_PR5:
		strcpy(ver, "PR5");
		break;
	case CMCS_CTP_F917_MP1:
		strcpy(ver, "MP1");
		break; 		
	case CMCS_CTP_F917_MP2:
		strcpy(ver, "MP2");
		break; 		
	case CMCS_CTP_F917_MP3:
		strcpy(ver, "MP3");
		break;
       /* FIH, Debbie, 2010/05/24 { */
	case CMCS_125_4G4G_FAA_PR1:
		strcpy(ver, "PR1_125");
		break;
	case CMCS_125_4G4G_FAA_PR2:
		strcpy(ver, "PR2_125");
		break;
	case CMCS_125_4G4G_FAA_PR3:
		strcpy(ver, "PR3_125");
		break;
	case CMCS_125_4G4G_FAA_MP1:
		strcpy(ver, "MP1_125");
		break;
	case CMCS_128_4G4G_FAA_PR1:
		strcpy(ver, "PR1_128");
		break;
	case CMCS_128_4G4G_FAA_PR2:
		strcpy(ver, "PR2_128");
		break;
	case CMCS_128_4G4G_FAA_PR3:
		strcpy(ver, "PR3_128");
		break;
	case CMCS_128_4G4G_FAA_MP1:
		strcpy(ver, "MP1_128");
		break;
	/* FIH, Debbie, 2010/05/24 } */
	/* FIH, Debbie, 2010/05/04 { */
	case CMCS_7627_F20_PR1:
		strcpy(ver, "PR1");
		break; 	
	case CMCS_7627_F20_PR2:
		strcpy(ver, "PR2");
		break; 	
	case CMCS_7627_F20_PR3:
		strcpy(ver, "PR3");
		break; 	
	case CMCS_7627_F20_MP1:
		strcpy(ver, "MP1");
		break;
	/* FIH, Debbie, 2010/05/04 } */
	default:
		strcpy(ver, "Unkonwn Baseband version");
		break;
	}

	len = snprintf(page, PAGE_SIZE, "%s\n",
		ver);

	return proc_calc_metrics(page, start, off, count, eof, len);
}

static int serial_number_read_proc(char *page, char **start, off_t off,
                 int count, int *eof, void *data)
{
    int len;
    extern char *board_serial;

    len = snprintf(page, PAGE_SIZE, "%s\n", board_serial);
    return proc_calc_metrics(page, start, off, count, eof, len);
}

static int device_HWSpec_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len;
	//int HWID = FIH_READ_HWID_FROM_SMEM();
	int HWID = FIH_READ_ORIG_HWID_FROM_SMEM();
	char ver[24];
		
	switch (HWID){
	case CMCS_HW_VER_EVB1:
		strcpy(ver, "R-_-R");
		break; 
	case CMCS_HW_VER_EVB2:
		strcpy(ver, "R-_-R");
		break; 
	case CMCS_HW_VER_EVB3:
		strcpy(ver, "R-_-R");
		break; 
	case CMCS_ORIG_RTP_PR1:
		if (JogballExist_pr1)
			strcpy(ver, "R-J-R");
		else
			strcpy(ver, "R-_-R");
		break; 
	case CMCS_850_RTP_PR2:
		if (JogballExist_pr2)
			strcpy(ver, "R-J-R");
		else
			strcpy(ver, "R-_-R");
		break;    
	case CMCS_850_RTP_PR3:
		if (JogballExist_pr2)
			strcpy(ver, "R-J-R");
		else
			strcpy(ver, "R-_-R");
		break; 
	case CMCS_850_RTP_MP1:
		if (JogballExist_pr2)
			strcpy(ver, "R-J-R");
		else
			strcpy(ver, "R-_-R");
		break;    
	case CMCS_850_RTP_MP2:
		if (JogballExist_pr2)
			strcpy(ver, "R-J-R");
		else
			strcpy(ver, "R-_-R");
		break; 
	case CMCS_850_RTP_MP3:
		if (JogballExist_pr2)
			strcpy(ver, "R-J-R");
		else
			strcpy(ver, "R-_-R");
		break; 
	case CMCS_ORIG_CTP_PR1:
		if (JogballExist_pr1)
			strcpy(ver, "M-J-C");
		else
			strcpy(ver, "M-_-C");
		break; 
	case CMCS_850_CTP_PR2:
		if (JogballExist_pr2)
			strcpy(ver, "M-J-C");
		else
			strcpy(ver, "M-_-C");
		break;    
	case CMCS_850_CTP_PR3:
		if (JogballExist_pr2)
			strcpy(ver, "M-J-C");
		else
			strcpy(ver, "M-_-C");
		break; 
	case CMCS_850_CTP_MP1:
		if (JogballExist_pr2)
			strcpy(ver, "M-J-C");
		else
			strcpy(ver, "M-_-C");
		break;    
	case CMCS_850_CTP_MP2:
		if (JogballExist_pr2)
			strcpy(ver, "M-J-C");
		else
			strcpy(ver, "M-_-C");
		break; 
	case CMCS_850_CTP_MP3:
		if (JogballExist_pr2)
			strcpy(ver, "M-J-C");
		else
			strcpy(ver, "M-_-C");
		break; 
	case CMCS_900_RTP_PR2:
		if (JogballExist_pr2)
			strcpy(ver, "R-J-R");
		else
			strcpy(ver, "R-_-R");
		break;    
	case CMCS_900_RTP_PR3:
		if (JogballExist_pr2)
			strcpy(ver, "R-J-R");
		else
			strcpy(ver, "R-_-R");
		break; 
	case CMCS_900_RTP_MP1:
		if (JogballExist_pr2)
			strcpy(ver, "R-J-R");
		else
			strcpy(ver, "R-_-R");
		break;    
	case CMCS_900_RTP_MP2:
		if (JogballExist_pr2)
			strcpy(ver, "R-J-R");
		else
			strcpy(ver, "R-_-R");
		break; 
	case CMCS_900_RTP_MP3:
		if (JogballExist_pr2)
			strcpy(ver, "R-J-R");
		else
			strcpy(ver, "R-_-R");
		break; 
	case CMCS_900_CTP_PR2:
		if (JogballExist_pr2)
			strcpy(ver, "M-J-C");
		else
			strcpy(ver, "M-_-C");
		break;    
	case CMCS_900_CTP_PR3:
		if (JogballExist_pr2)
			strcpy(ver, "M-J-C");
		else
			strcpy(ver, "M-_-C");
		break; 
	case CMCS_900_CTP_MP1:
		if (JogballExist_pr2)
			strcpy(ver, "M-J-C");
		else
			strcpy(ver, "M-_-C");
		break;    
	case CMCS_900_CTP_MP2:
		if (JogballExist_pr2)
			strcpy(ver, "M-J-C");
		else
			strcpy(ver, "M-_-C");
		break; 
	case CMCS_900_CTP_MP3:
		if (JogballExist_pr2)
			strcpy(ver, "M-J-C");
		else
			strcpy(ver, "M-_-C");
		break; 
	case CMCS_145_CTP_PR1:
		strcpy(ver,  "M-J-C");
		break; 
	case CMCS_125_FST_PR1:
		strcpy(ver, "M-_-C");
		break; 
	case CMCS_125_FST_PR2:
		strcpy(ver, "M-_-C");
		break;    
	case CMCS_125_FST_MP1:
		strcpy(ver, "M-_-C");
		break; 
	case CMCS_128_FST_PR1:
		strcpy(ver, "M-_-C");
		break; 
	case CMCS_128_FST_PR2:
		strcpy(ver, "M-_-C");
		break;    
	case CMCS_128_FST_MP1:
		strcpy(ver, "M-_-C");
		break; 
	case CMCS_125_CTP_GRE_PR1:
	case CMCS_125_CTP_GRE_PR2:
	case CMCS_125_CTP_GRE_MP1:
	case CMCS_125_CTP_GRE_MP2:
		if (JogballExist_pr2)
			strcpy(ver, "M-J-C");
		else
			strcpy(ver, "M-_-C");
		break;
	/* FIH, Debbie, 2010/05/04 { */
	case CMCS_125_FA9_PR1:
	case CMCS_125_FA9_PR2:
	case CMCS_125_FA9_PR3:
	case CMCS_125_FA9_MP1:
		strcpy(ver, "M-_-C");
		break;
	/* FIH, Debbie, 2010/05/04 } */
	case CMCS_7627_ORIG_EVB1:
		strcpy(ver, "M-_-C");
		break;    
	case CMCS_7627_F905_PR1:
	case CMCS_7627_F905_PR2:
	case CMCS_7627_F905_PR3:
		strcpy(ver, "M-_-C");
		break; 
	case CMCS_7627_F913_PR1:
	case CMCS_7627_F913_PR2: 
	case CMCS_7627_F913_PR3: 		
	case CMCS_7627_F913_MP1_W:
	case CMCS_7627_F913_MP1_C_G:
	/* FIH, Debbie, 2010/05/24 { */
	case CMCS_7627_F913_MP1_W_4G4G:
	case CMCS_7627_F913_MP1_C_G_4G4G:
	/* FIH, Debbie, 2010/05/24 } */
		strcpy(ver, "M-_-C");
		break;
	/* FIH, Debbie, 2010/05/04 { */	
	case CMCS_CTP_F917_PR1:	
	case CMCS_CTP_F917_PR2:		
	case CMCS_CTP_F917_PR3:	
	case CMCS_CTP_F917_PR4:	
	case CMCS_CTP_F917_PR5:
	case CMCS_CTP_F917_MP1:	
	case CMCS_CTP_F917_MP2:	
	case CMCS_CTP_F917_MP3:
		strcpy(ver, "M-_-C");
		break; 	
	/* FIH, Debbie, 2010/05/24 { */
	case CMCS_125_4G4G_FAA_PR1:
	case CMCS_125_4G4G_FAA_PR2:
	case CMCS_125_4G4G_FAA_PR3:
	case CMCS_125_4G4G_FAA_MP1:
	case CMCS_128_4G4G_FAA_PR1:
	case CMCS_128_4G4G_FAA_PR2:
	case CMCS_128_4G4G_FAA_PR3:
	case CMCS_128_4G4G_FAA_MP1:
		strcpy(ver, "M-_-C");
		break; 	
        /* FIH, Debbie, 2010/05/24 } */
	case CMCS_7627_F20_PR1:
	case CMCS_7627_F20_PR2:
	case CMCS_7627_F20_PR3:
	case CMCS_7627_F20_MP1:
		strcpy(ver, "M-_-C");
		break;
	/* FIH, Debbie, 2010/05/04 } */
	default:
		strcpy(ver, "Unkonwn HW Spec");
		break;
	}

	len = snprintf(page, PAGE_SIZE, "%s\n",
		ver);
		
	return proc_calc_metrics(page, start, off, count, eof, len);	
}
#ifdef POWER_ON_CAUSE_PROC_READ_ENTRY
static int proc_read_power_on_cause(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len;
	uint32_t poc = FIH_READ_POWER_ON_CAUSE();

	len = snprintf(page, PAGE_SIZE, "0x%x\n", poc);

	return proc_calc_metrics(page, start, off, count, eof, len);	
}
#endif
static struct {
		char *name;
		int (*read_proc)(char*,char**,off_t,int,int*,void*);
} *p, adq_info[] = {
	{"socinfo",	build_version_read_proc},
	{"devmodel",	device_model_read_proc},
	{"baseband",	baseband_read_proc},
	{"serialnumber",serial_number_read_proc},
	{"hwspec",	device_HWSpec_read_proc},
	#ifdef POWER_ON_CAUSE_PROC_READ_ENTRY
	{"poweroncause",	proc_read_power_on_cause},
	#endif
	{NULL,},
};

void adq_info_init(void)
{	
	JogballExist_pr1 = 0;
	JogballExist_pr2 = 0;
	if(!gpio_get_value(PLUS_X_GPIO) || !gpio_get_value(NEG_X_GPIO) ||
	    !gpio_get_value(PLUS_Y_GPIO) || !gpio_get_value(NEG_Y_GPIO)) 
	{
		JogballExist_pr1 = 1;
		printk("PLUS_X_GPIO=%d\n",gpio_get_value(PLUS_X_GPIO));
		printk("NEG_X_GPIO=%d\n",gpio_get_value(NEG_X_GPIO));
		printk("PLUS_Y_GPIO=%d\n",gpio_get_value(PLUS_Y_GPIO));
		printk("NEG_Y_GPIOd=%d\n",gpio_get_value(NEG_Y_GPIO));
		printk("JogballExist1=%d\n",JogballExist_pr1);
	}
	if(!gpio_get_value(NEG_Y_GPIO)) 
	{
		JogballExist_pr2 = 1;
		printk("JogballExist2=%d\n",JogballExist_pr2);
	}
		
	for (p = adq_info; p->name; p++)
		create_proc_read_entry(p->name, 0, NULL, p->read_proc, NULL);
		
}
EXPORT_SYMBOL(adq_info_init);

void adq_info_remove(void)
{
	for (p = adq_info; p->name; p++)
		remove_proc_entry(p->name, NULL);
}
EXPORT_SYMBOL(adq_info_remove);
