#include "car_core.h"
#include <stdio.h>
#include <stdlib.h>
static unsigned char gray[CAR_W*CAR_H],binary[CAR_W*CAR_H];
int main(int argc,char **argv){
 if(argc!=3)return 2;FILE *f=fopen(argv[1],"rb");if(!f)return 3;
 char line[128];for(int i=0;i<3;i++)if(!fgets(line,sizeof line,f))return 4;
 if(fread(gray,1,sizeof gray,f)!=sizeof gray)return 5;fclose(f);
 vision_state_t state;vision_config_t cfg;vision_result_t result;
 vision_reset(&state);vision_config_default(&cfg);vision_process(&state,&cfg,gray,binary,&result);
 f=fopen(argv[2],"w");if(!f)return 6;
 fprintf(f,"valid,%u,error,%.4f,threshold,%u,paired,%u,scanned,%u\n",result.valid,result.error,result.threshold,result.paired_rows,result.scanned_rows);
 for(int y=0;y<CAR_H;y++)fprintf(f,"%d,%d,%d,%d,%u\n",y,result.left[y],result.right[y],result.center[y],result.edge_flags[y]);
 fclose(f);return 0;
}
