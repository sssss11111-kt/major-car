#include "car_core.h"
#include <stdio.h>
#include <stdlib.h>
int main(int argc,char**argv){unsigned char g[CAR_W*CAR_H],b[CAR_W*CAR_H];char header[100];FILE*f=fopen(argv[1],"rb");if(!f)return 2;for(int i=0;i<3;i++)if(!fgets(header,sizeof header,f))return 3;if(fread(g,1,sizeof g,f)!=sizeof g)return 4;fclose(f);vision_state_t s;vision_config_t c;vision_result_t r;vision_reset(&s);vision_config_default(&c);for(int n=0;n<4;n++){vision_process(&s,&c,g,b,&r);printf("frame=%d valid=%d pair=%u/%u reject=%u error=%.2f threshold=%u\n",n,r.valid,r.paired_rows,r.scanned_rows,r.rejected_rows,r.error,r.threshold);}return 0;}
