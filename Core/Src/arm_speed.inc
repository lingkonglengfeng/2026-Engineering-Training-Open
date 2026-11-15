/* Included once by arm_start_pose.c: access to the existing pose owner avoids
 * a second motor/storage task and keeps all parameter writes serialized. */
typedef struct {uint32_t magic;uint16_t version,size;uint32_t sequence;
    uint16_t rates[ARM_SPEED_POINT_COUNT-1U][ARM_SPEED_AXIS_COUNT];uint16_t reserved;uint32_t crc;} speed_record_t;
static uint16_t s_speed_rates[ARM_SPEED_POINT_COUNT-1U][ARM_SPEED_AXIS_COUNT]={
    {1000,1000,1000,1000,1000},{1000,1000,1000,1000,1000},{1000,1000,1000,1000,1000},
    {1000,1000,1000,1000,1000},{1000,1000,1000,1000,1000},{1000,1000,1000,1000,1000},{1000,1000,1000,1000,1000}};
static uint32_t s_speed_sequence;
static uint8_t s_speed_flash;
static float s_speed_timeout_scale=1.0f;
static const char *const speed_names[]={"wait","grab","tray","traypick","ring1","ring2","ring3"};
static uint8_t Speed_Point(const char *name)
{uint8_t i;for(i=0;i<ARM_SPEED_POINT_COUNT-1U;i++)if(!strcmp(name,speed_names[i]))return i+1U;return 0U;}
static void Speed_RefreshTimeout(void)
{uint8_t p,a;uint16_t slowest=1000U;for(p=0;p<ARM_SPEED_POINT_COUNT-1U;p++)for(a=0;a<ARM_SPEED_AXIS_COUNT;a++)if(s_speed_rates[p][a]<slowest)slowest=s_speed_rates[p][a];s_speed_timeout_scale=1000.0f/slowest;}
void ArmSpeed_Init(void)
{uint8_t p,a;for(p=0;p<ARM_SPEED_POINT_COUNT-1U;p++)for(a=0;a<ARM_SPEED_AXIS_COUNT;a++)s_speed_rates[p][a]=1000U;s_speed_sequence=0U;s_speed_flash=0U;s_speed_timeout_scale=1.0f;}
uint8_t ArmSpeed_ActivePoint(void)
{
    arm_camera_state_t camera=ArmCalibration_GetCameraState();
    if(s_speed_forced)return s_speed_forced;
    if(camera>=ARM_CAMERA_PRECHECK && camera<=ARM_CAMERA_LIFT_FINAL)return ARM_SPEED_WAIT;
    if(ArmStartPose_IsActive() || (ArmRound1_IsActive() && s_pick)) {
        if(s_pick==1U)return ARM_SPEED_TRAY_PICK;
        if(s_pick==2U)return ARM_SPEED_TRAY;
        if(s_pick==3U || s_pick==4U)return (uint8_t)(ARM_SPEED_RING1+s_ring_station-1U);
        if(s_material_direct)return ArmMaterial_IsActive()?ArmMaterial_SpeedPoint():ARM_SPEED_GRAB;
        if(s_generic)return s_generic_speed_point;
        return ARM_SPEED_WAIT;
    }
    if(ArmMaterial_IsActive())return ArmMaterial_SpeedPoint();
    return ARM_SPEED_NONE;
}
float ArmSpeed_Factor(uint8_t axis)
{return ArmSpeed_PointFactor(ArmSpeed_ActivePoint(),axis);}
float ArmSpeed_PointFactor(uint8_t p,uint8_t axis)
{return p && p<ARM_SPEED_POINT_COUNT && axis<ARM_SPEED_AXIS_COUNT?(float)s_speed_rates[p-1U][axis]/1000.0f:1.0f;}
float ArmSpeed_MaxFactor(uint8_t axis)
{uint8_t p;float f=1.0f;if(axis<ARM_SPEED_AXIS_COUNT)for(p=0;p<ARM_SPEED_POINT_COUNT-1U;p++)f=fmaxf(f,s_speed_rates[p][axis]*.001f);return f;}
uint16_t ArmSpeed_Rpm(uint8_t axis,uint16_t base)
{return base?(uint16_t)fmaxf(1.0f,fminf(ZDT_CAN_RUN_MAX_RPM,roundf(base*ArmSpeed_Factor(axis)))):0U;}
uint16_t ArmSpeed_AllowedRpm(uint8_t axis,uint16_t base)
{return base?(uint16_t)fmaxf(1.0f,fminf(ZDT_CAN_RUN_MAX_RPM,roundf(base*ArmSpeed_MaxFactor(axis)))):0U;}
uint32_t ArmSpeed_Timeout(uint32_t base)
{return (uint32_t)fmin((double)base*s_speed_timeout_scale,4294967295.0);}
static uint32_t Speed_Address(uint8_t bank)
{return W25Q64_GetCapacity()-(COMPETITION_SPEED_BANK_SECTORS-bank)*W25Q64_SECTOR_SIZE;}
static uint8_t Speed_Valid(const speed_record_t *r)
{
    uint8_t p,a;
    if(r->magic!=0x53504431UL || r->version!=1U || r->size!=sizeof(*r) ||
       r->crc!=Disc_CrcBytes(r,offsetof(speed_record_t,crc)))return 0U;
    for(p=0;p<ARM_SPEED_POINT_COUNT-1U;p++)for(a=0;a<ARM_SPEED_AXIS_COUNT;a++)
        if(r->rates[p][a]<100U || r->rates[p][a]>3000U)return 0U;
    return 1U;
}
static int Speed_Newest(speed_record_t r[2])
{
    uint8_t a,b;
    if(W25Q64_GetCapacity()<COMPETITION_SPEED_BANK_SECTORS*W25Q64_SECTOR_SIZE ||
       W25Q64_Read(Speed_Address(0),(uint8_t *)&r[0],sizeof(r[0]))!=W25Q64_OK ||
       W25Q64_Read(Speed_Address(1),(uint8_t *)&r[1],sizeof(r[1]))!=W25Q64_OK)return -2;
    a=Speed_Valid(&r[0]);b=Speed_Valid(&r[1]);
    return a && b?((int32_t)(r[1].sequence-r[0].sequence)>0?1:0):b?1:a?0:-1;
}
HAL_StatusTypeDef ArmSpeed_Load(void)
{
    speed_record_t r[2];int bank;HAL_StatusTypeDef result=HAL_ERROR;
    if(!W25Q64_StorageTryLock())return HAL_BUSY;
    if(W25Q64_Init(NULL)==W25Q64_OK) {
        bank=Speed_Newest(r);
        if(bank>=0){memcpy(s_speed_rates,r[bank].rates,sizeof(s_speed_rates));Speed_RefreshTimeout();s_speed_sequence=r[bank].sequence;s_speed_flash=1U;result=HAL_OK;}
        /* Empty/invalid banks leave the current RAM values intact. At boot
         * Init already installed100%; an explicit failed load cannot erase edits. */
    }
    W25Q64_StorageUnlock();return result;
}
static HAL_StatusTypeDef Speed_Save(void)
{
    speed_record_t r[2],record;uint8_t verify[sizeof(record)],target;int bank;HAL_StatusTypeDef result=HAL_ERROR;
    if(!W25Q64_StorageTryLock())return HAL_BUSY;
    if(W25Q64_Init(NULL)==W25Q64_OK && (bank=Speed_Newest(r))!=-2) {
        target=(uint8_t)(bank==0?1:0);memset(&record,0,sizeof(record));record.magic=0x53504431UL;
        record.version=1U;record.size=sizeof(record);record.sequence=bank<0?1:r[bank].sequence+1U;
        memcpy(record.rates,s_speed_rates,sizeof(record.rates));record.crc=Disc_CrcBytes(&record,offsetof(speed_record_t,crc));
        if(Speed_Valid(&record) && W25Q64_EraseSector(Speed_Address(target))==W25Q64_OK &&
           W25Q64_WriteVerified(Speed_Address(target),(const uint8_t *)&record,sizeof(record),verify,sizeof(verify))==W25Q64_OK) {
            s_speed_sequence=record.sequence;s_speed_flash=1U;result=HAL_OK;
        }
    }
    W25Q64_StorageUnlock();return result;
}
static void Speed_Print(void)
{
    uint8_t p;char line[192];
    snprintf(line,sizeof(line),"[ARM RATES BEGIN] v=1 firmware=292 variant=" COMPETITION_VARIANT " source=%s sequence=%lu min=10 max=300\r\n",s_speed_flash?"W25Q64":"RAM",(unsigned long)s_speed_sequence);ZDT_Console_Print(line);
    for(p=0;p<ARM_SPEED_POINT_COUNT-1U;p++) {
        snprintf(line,sizeof(line),"[ARM RATE] point=%s lift=%.1f reach=%.1f dm=%.1f grip=%.1f table=%.1f\r\n",speed_names[p],
            s_speed_rates[p][0]*.1,s_speed_rates[p][1]*.1,s_speed_rates[p][2]*.1,s_speed_rates[p][3]*.1,s_speed_rates[p][4]*.1);ZDT_Console_Print(line);
    }
    ZDT_Console_Print("[ARM RATES END]\r\n");
}
uint8_t ArmSpeed_Command(const char *line)
{
    float values[5];uint16_t rates[5];uint8_t p,a;char name[12],tail,out[128];HAL_StatusTypeDef result=HAL_ERROR;
    if(strncmp(line,"arm rates",9U) || (line[9] && line[9]!=' '))return 0U;
    if(!strcmp(line,"arm rates") || !strcmp(line,"arm rates status")){Speed_Print();return 1U;}
    if(ArmStartPose_IsActive() || ArmMaterial_IsActive() || ArmRound1_IsActive() ||
       ArmCalibration_GetState()!=ARM_CAL_IDLE ||
       (ArmCalibration_GetCameraState()>=ARM_CAMERA_PRECHECK && ArmCalibration_GetCameraState()<=ARM_CAMERA_LIFT_FINAL) || ChassisMotor_HasMotionDemand())result=HAL_BUSY;
    else if(sscanf(line,"arm rates set %11s %f %f %f %f %f %c",name,&values[0],&values[1],&values[2],&values[3],&values[4],&tail)==6 && (p=Speed_Point(name))!=0U) {
        for(a=0;a<5U;a++)if(!isfinite(values[a]) || values[a]<10.0f || values[a]>300.0f)break;
        if(a==5U){for(a=0;a<5U;a++)rates[a]=(uint16_t)lroundf(values[a]*10.0f);memcpy(s_speed_rates[p-1U],rates,sizeof(rates));Speed_RefreshTimeout();s_speed_flash=0U;result=HAL_OK;}
    } else if(!strcmp(line,"arm rates save"))result=Speed_Save();
    else if(!strcmp(line,"arm rates load"))result=ArmSpeed_Load();
    else if(!strcmp(line,"arm rates defaults")){ArmSpeed_Init();result=HAL_OK;}
    snprintf(out,sizeof(out),"[ARM RATES ACK] op=%s HAL=%u; no motion\r\n",strstr(line," set ")?"set":strstr(line,"save")?"save":strstr(line,"load")?"load":strstr(line,"defaults")?"defaults":"invalid",(unsigned)result);ZDT_Console_Print(out);
    return 1U;
}
