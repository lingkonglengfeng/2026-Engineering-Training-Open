void RatesOnlyTests(void)
{
    unsigned p,a;uint32_t moves,writes;uint8_t i;speed_record_t records[2];int bank;
    ArmStartPose_Init();memset(flash,255,sizeof(flash));chassis=0;s_phase=POSE_IDLE;
    moves=dm_moves+linear_moves;writes=flash_writes;
    CHECK(ArmSpeed_Load()==HAL_ERROR);
    for(p=1;p<ARM_SPEED_POINT_COUNT;p++)for(a=0;a<ARM_SPEED_AXIS_COUNT;a++)CHECK(ArmSpeed_PointFactor(p,a)==1.0f);
    s_phase=POSE_DM;s_pick=1;CHECK(ArmSpeed_ActivePoint()==ARM_SPEED_TRAY_PICK);
    s_pick=2;CHECK(ArmSpeed_ActivePoint()==ARM_SPEED_TRAY);
    s_pick=3;s_ring_station=2;CHECK(ArmSpeed_ActivePoint()==ARM_SPEED_RING2);
    s_pick=4;s_ring_station=3;CHECK(ArmSpeed_ActivePoint()==ARM_SPEED_RING3);
    s_pick=0;s_generic=1;s_generic_speed_point=ARM_SPEED_GRAB;CHECK(ArmSpeed_ActivePoint()==ARM_SPEED_GRAB);
    s_generic=0;CHECK(ArmSpeed_ActivePoint()==ARM_SPEED_WAIT);
    s_phase=POSE_IDLE;CHECK(ArmSpeed_ActivePoint()==ARM_SPEED_NONE);
    CHECK(ArmSpeed_Command("arm rates set grab 120 150 110 80 70"));
    CHECK(strstr(last_console,"HAL=0"));s_speed_forced=ARM_SPEED_GRAB;
    CHECK(fabsf(ArmSpeed_Factor(ARM_SPEED_LIFT)-1.2f)<.001f);
    CHECK(ArmSpeed_Rpm(ARM_SPEED_REACH,262)==393);
    CHECK(ArmSpeed_Rpm(ARM_SPEED_LIFT,410)==492);
    CHECK(ArmSpeed_AllowedRpm(ARM_SPEED_REACH,246)==369);
    CHECK(ArmSpeed_Timeout(1000)>=1428 && ArmSpeed_Timeout(1000)<=1429);
    s_speed_forced=0;CHECK(ArmSpeed_Factor(ARM_SPEED_REACH)==1.0f);
    CHECK(ArmSpeed_Command("arm rates set ring2 50 60 70 80 90"));
    CHECK(ArmSpeed_PointFactor(ARM_SPEED_RING1,ARM_SPEED_LIFT)==1.0f);
    CHECK(ArmSpeed_PointFactor(ARM_SPEED_RING2,ARM_SPEED_LIFT)==.5f);
    CHECK(ArmSpeed_Command("arm rates set grab 120 150 nan 80 70"));CHECK(strstr(last_console,"HAL=1"));
    CHECK(ArmSpeed_Command("arm rates set grab 120 150 301 80 70"));CHECK(strstr(last_console,"HAL=1"));
    CHECK(ArmSpeed_PointFactor(ARM_SPEED_GRAB,ARM_SPEED_REACH)==1.5f);
    chassis=1;CHECK(ArmSpeed_Command("arm rates set grab 100 100 100 100 100"));CHECK(strstr(last_console,"HAL=2"));chassis=0;
    CHECK(dm_moves+linear_moves==moves && flash_writes==writes);
    CHECK(ArmSpeed_Command("arm rates save"));CHECK(strstr(last_console,"HAL=0") && flash_writes==writes+1);
    {speed_record_t other;uint32_t address=W25Q64_GetCapacity()-(COMPETITION_PRELIMINARY?22U:20U)*W25Q64_SECTOR_SIZE;
     memset(&other,0,sizeof(other));other.magic=0x53504431UL;other.version=1;other.size=sizeof(other);other.sequence=999;
     for(p=0;p<ARM_SPEED_POINT_COUNT-1U;p++)for(a=0;a<ARM_SPEED_AXIS_COUNT;a++)other.rates[p][a]=2600;
     other.crc=Disc_CrcBytes(&other,offsetof(speed_record_t,crc));memcpy(flash+at(address,sizeof(other)),&other,sizeof(other));}
    ArmSpeed_Init();CHECK(ArmSpeed_Load()==HAL_OK);CHECK(ArmSpeed_PointFactor(ARM_SPEED_GRAB,ARM_SPEED_REACH)==1.5f);
    CHECK(Speed_Newest(records)>=0);bank=Speed_Newest(records);
    CHECK(ArmSpeed_Command("arm rates set grab 100 170 100 100 100"));CHECK(ArmSpeed_Command("arm rates save"));
    /* Corrupt latest CRC: previous complete bank remains readable. */
    bank=Speed_Newest(records);CHECK(bank>=0);flash[at(Speed_Address((uint8_t)bank),1)]^=1U;
    ArmSpeed_Init();CHECK(ArmSpeed_Load()==HAL_OK);CHECK(ArmSpeed_PointFactor(ARM_SPEED_GRAB,ARM_SPEED_REACH)==1.5f);
    /* Failed write and BUSY cannot claim a durable update or erase the old bank. */
    flash_fail=1;CHECK(ArmSpeed_Command("arm rates save"));CHECK(strstr(last_console,"HAL=1"));flash_fail=0;
    flash_busy=1;CHECK(ArmSpeed_Command("arm rates save"));CHECK(strstr(last_console,"HAL=2"));flash_busy=0;
    for(i=0;i<2;i++)memset(flash+at(Speed_Address(i),4096),255,4096);
    CHECK(ArmSpeed_Load()==HAL_ERROR);CHECK(ArmSpeed_PointFactor(ARM_SPEED_GRAB,ARM_SPEED_REACH)==1.5f);
    CHECK(dm_moves+linear_moves==moves);
    CHECK(ArmSpeed_Command("arm rates defaults"));
    for(p=1;p<ARM_SPEED_POINT_COUNT;p++)for(a=0;a<ARM_SPEED_AXIS_COUNT;a++)CHECK(ArmSpeed_PointFactor(p,a)==1.0f);
}
