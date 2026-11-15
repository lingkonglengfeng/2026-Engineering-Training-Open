/* Production storage, shared trims and generic move; mocked hardware ports. */
static void step(void);
static int test_profile_isolation(void)
{
#if COMPETITION_PRELIMINARY
    unsigned i;arm_ui_record_t legacy,records[2];disc_record_t final_disc;
    uint32_t moves,writes;uint8_t final_before[4U*4096U];
    memset(axes,0,sizeof(axes));memset(flash,255,sizeof(flash));flash_busy=flash_fail=0;
    for(i=0;i<2;i++){axes[i].travel_mm=i?178.0f:240.0f;axes[i].zero_valid=1;axes[i].position_sequence=axes[i].flags_sequence=1;axes[i].position_tick=axes[i].flags_tick=tick;axes[i].flags=0x81;}
    motor.fb.fb_count=1;motor.fb.last_tick=tick;motor.fb.vel_rads=0;chassis=0;cal_state=ARM_CAL_IDLE;ArmStartPose_Init();
    memset(&legacy,0,sizeof(legacy));legacy.magic=ARM_UI_MAGIC;legacy.version=1;legacy.size=sizeof(legacy);legacy.sequence=7;
    legacy.servos[0]=225;legacy.servos[1]=308;legacy.servos[2]=19;legacy.servos[3]=109;legacy.servos[4]=199;legacy.crc=Pose_ParamsCrc(&legacy);
    memcpy(flash+at(W25Q64_GetCapacity()-8U*4096U,sizeof(legacy)),&legacy,sizeof(legacy));
    /* Realistic FINAL records coexist; PRELIM boot must never adopt them. */
    legacy.servos[0]=159;legacy.trim[0][0][0]=-10;legacy.version=2;legacy.crc=Pose_ParamsCrc(&legacy);
    memcpy(flash+at(W25Q64_GetCapacity()-12U*4096U,sizeof(legacy)),&legacy,sizeof(legacy));
    memset(&final_disc,0,sizeof(final_disc));final_disc.magic=DISC_MAGIC;final_disc.version=2;final_disc.size=sizeof(final_disc);
    final_disc.trim[0][1]=15;final_disc.crc=Disc_Crc(&final_disc);
    memcpy(flash+at(W25Q64_GetCapacity()-10U*4096U,sizeof(final_disc)),&final_disc,sizeof(final_disc));
    memcpy(final_before,flash+at(W25Q64_GetCapacity()-12U*4096U,sizeof(final_before)),sizeof(final_before));
    writes=flash_writes;moves=linear_moves;
    CHECK(Pose_ParamsAddress(0)==W25Q64_GetCapacity()-8U*4096U);
    CHECK(Disc_Address(0)==W25Q64_GetCapacity()-14U*4096U);
    CHECK(ArmStartPose_LoadParameters()==HAL_OK && servo_parameters[0]==225 && s_ring_trim[0][0]==0);
    CHECK(s_disc_trim[0][1]==0 && !s_disc_flash_source && flash_writes==writes);
    s_disc_trim[0][1]=-12;s_ring_trim[0][0]=1;servo_parameters[0]=230;
    CHECK(Pose_ParamsSave() && Disc_Save());
    CHECK(Pose_ParamsNewestAt(records,8)>=0 && records[0].version==1);
    ArmStartPose_Init();CHECK(ArmStartPose_LoadParameters()==HAL_OK);
    CHECK(s_disc_trim[0][1]==-12 && s_ring_trim[0][0]==1 && servo_parameters[0]==230);
    CHECK(ArmStartPose_Command("arm profile final")==1 && strstr(last_console,"READY target=FINAL active=PRELIM"));
    CHECK(s_disc_trim[0][1]==-12 && s_ring_trim[0][0]==1 && servo_parameters[0]==230);
    CHECK(!memcmp(final_before,flash+at(W25Q64_GetCapacity()-12U*4096U,sizeof(final_before)),sizeof(final_before)) && linear_moves==moves);
    s_disc_trim[0][1]=0;CHECK(ArmStartPose_Command("arm profile prelim")==1 && s_disc_trim[0][1]==-12);
    CHECK(strstr(last_console,"READY target=PRELIM active=PRELIM"));
    flash_fail=1;CHECK(ArmStartPose_Command("arm profile final")==1 && strstr(last_console,"FAILED"));flash_fail=0;
    CHECK(!memcmp(final_before,flash+at(W25Q64_GetCapacity()-12U*4096U,sizeof(final_before)),sizeof(final_before)) && linear_moves==moves);
    return failures;
#else
    unsigned i;arm_ui_record_t legacy,records[2];int bank;uint32_t moves,writes;
    uint8_t preliminary_poses[2U*4096U];
    memset(axes,0,sizeof(axes));memset(flash,255,sizeof(flash));flash_busy=flash_fail=0;
    for(i=0;i<2;i++){axes[i].travel_mm=i?178.0f:240.0f;axes[i].zero_valid=1;axes[i].position_sequence=axes[i].flags_sequence=1;axes[i].position_tick=axes[i].flags_tick=tick;axes[i].flags=0x81;}
    motor.fb.fb_count=1;motor.fb.last_tick=tick;motor.fb.vel_rads=0;chassis=0;cal_state=ARM_CAL_IDLE;ArmStartPose_Init();
    memset(&legacy,0,sizeof(legacy));legacy.magic=ARM_UI_MAGIC;legacy.version=1;legacy.size=sizeof(legacy);legacy.sequence=7;
    for(i=0;i<3;i++)legacy.trim[0][i][0]=-10;
    memcpy(legacy.trim[1],legacy.trim[0],sizeof(legacy.trim[0]));
    legacy.servos[0]=159;legacy.servos[1]=308;legacy.servos[2]=19;legacy.servos[3]=109;legacy.servos[4]=199;legacy.crc=Pose_ParamsCrc(&legacy);
    memcpy(flash+at(W25Q64_GetCapacity()-8U*4096U,sizeof(legacy)),&legacy,sizeof(legacy));
    memset(preliminary_poses,0xA5,sizeof(preliminary_poses));
    memcpy(flash+at(W25Q64_GetCapacity()-14U*4096U,sizeof(preliminary_poses)),preliminary_poses,sizeof(preliminary_poses));
    writes=flash_writes;moves=linear_moves;
    CHECK(Pose_ParamsLoad()==HAL_OK && servo_parameters[0]==159 && s_ring_trim[0][0]==-10);
    CHECK(flash_writes==writes+1 && linear_moves==moves && !s_prelim_ready);
    CHECK(!memcmp(flash+at(W25Q64_GetCapacity()-8U*4096U,sizeof(legacy)),&legacy,sizeof(legacy)));
    s_disc_trim[ARM_DISC_WAIT][1]=15;
    CHECK(ArmStartPose_Command("arm profile prelim")==1 && s_prelim_ready && strstr(last_console,"READY target=PRELIM"));
    CHECK(servo_parameters[0]==159 && s_ring_trim[0][0]==-10 && linear_moves==moves);
    bank=Pose_ParamsNewestAt(records,8);CHECK(bank>=0 && records[bank].version==1 && records[bank].servos[0]==225);
    CHECK(records[bank].trim[0][0][0]==0 && fabsf(records[bank].trim[0][2][1]+5.59f)<.001f);
    legacy=records[bank];legacy.sequence=100;legacy.servos[0]=230;legacy.trim[0][0][0]=1;legacy.crc=Pose_ParamsCrc(&legacy);
    memcpy(flash+at(W25Q64_GetCapacity()-8U*4096U,sizeof(legacy)),&legacy,sizeof(legacy));
    CHECK(Profile_PreparePreliminary());bank=Pose_ParamsNewestAt(records,8);
    CHECK(bank==0 && records[bank].servos[0]==230 && records[bank].trim[0][0][0]==1);
    ArmStartPose_Init();CHECK(ArmStartPose_LoadParameters()==HAL_OK && s_prelim_ready);
    CHECK(servo_parameters[0]==159 && s_ring_trim[0][0]==-10 && s_disc_trim[ARM_DISC_WAIT][1]==15);
    servo_parameters[0]=180;s_ring_trim[0][0]=0;
    CHECK(ArmStartPose_Command("arm profile final")==1 && servo_parameters[0]==159 && s_ring_trim[0][0]==-10);
    CHECK(linear_moves==moves);
    CHECK(!memcmp(preliminary_poses,flash+at(W25Q64_GetCapacity()-14U*4096U,sizeof(preliminary_poses)),sizeof(preliminary_poses)));
    flash_fail=1;CHECK(!Profile_PreparePreliminary());flash_fail=0;
    bank=Pose_ParamsNewestAt(records,8);CHECK(bank==0 && records[bank].servos[0]==230);
    /* Corrupt private banks must not silently load the initial firmware area. */
    memset(flash+at(Pose_ParamsAddress(0),sizeof(legacy)),0,sizeof(legacy));
    memset(flash+at(Pose_ParamsAddress(1),sizeof(legacy)),0,sizeof(legacy));
    servo_parameters[0]=181;writes=flash_writes;
    CHECK(Pose_ParamsLoad()==HAL_ERROR && servo_parameters[0]==181 && flash_writes==writes && linear_moves==moves);
    return failures;
#endif
}
static int test_disc_profiles(void)
{
    unsigned i;float base[3],target[3],r,d,l;uint32_t moves,writes;start_profile_t launch;disc_record_t records[2];disc_record_v1_t old;int bank;
    memset(axes,0,sizeof(axes));memset(flash,255,sizeof(flash));flash_fail=flash_busy=0U;
    for(i=0;i<2;i++){axes[i].travel_mm=i?178.0f:240.0f;axes[i].zero_valid=1;axes[i].position_sequence=axes[i].flags_sequence=1;axes[i].position_tick=axes[i].flags_tick=tick;axes[i].flags=0x81;}
    motor.fb.fb_count=1;motor.fb.last_tick=tick;motor.fb.vel_rads=0;chassis=0;cal_state=ARM_CAL_IDLE;
    ArmStartPose_Init();launch=s_saved;moves=linear_moves;writes=flash_writes;
    CHECK(ArmStartPose_DiscTargets(ARM_DISC_WAIT,base,target)==HAL_OK && fabsf(target[0]+562.5272f)<.01f);
    CHECK(fabsf(target[1]+185.1581f)<.001f && fabsf(target[2]+2.0217f)<.001f);
    CHECK(ArmStartPose_DiscTargets(ARM_DISC_GRAB,base,target)==HAL_OK && target[1]<-280.0f);
    CHECK(Disc_TrimValid((const float (*)[3])s_disc_trim)); /* Default tray+1deg retains original reference allowance. */
    CHECK(ArmStartPose_Command("arm disc trim wait 2 -10 1")==1 && s_disc_trim[0][1]==-10);
    CHECK(ArmStartPose_Command("arm disc trim grab -3 -20 -1")==1 && s_disc_trim[1][0]==-3);
    CHECK(ArmStartPose_Command("arm disc trim tray -5 -6 2")==1 && s_disc_trim[2][1]==-6);
    CHECK(ArmStartPose_Command("arm disc trim traypick 2 5 1")==1 && s_disc_trim[3][1]==5);
    CHECK(ArmStartPose_Command("arm disc trim traypick 2 -10 1")==1 && s_disc_trim[3][1]==-10);
    CHECK(ArmStartPose_DiscTargets(ARM_DISC_TRAY_PICK,base,target)==HAL_OK && target[1]==-14 && fabsf(target[2]-163.825f)<.001f);
    CHECK(Pose_PickLiftDeg(&l)==HAL_OK && fabsf(l-target[0])<.001f);
    CHECK(ArmStartPose_Command("arm pick")==1 && s_pick==1U);
    CHECK(fabs(s_run.percent[0]-target[0]/360.0f*65536.0f/ARM_LIFT_FULL_TRAVEL_COUNTS*100.0f)<.001f);
    CHECK(fabs(s_run.percent[1]-target[1]/360.0f*65536.0f/ARM_REACH_FULL_TRAVEL_COUNTS*100.0f)<.001f && fabsf(s_run.dm_deg-target[2])<.001f);
    ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;
    CHECK(linear_moves==moves && flash_writes==writes && !memcmp(&launch,&s_saved,sizeof(launch)));
    CHECK(Pose_PlaceLiftBaseDeg(&l)==HAL_OK && Pose_PlaceLiftDeg(&r)==HAL_OK && fabsf(r-l+5)<.001f);
    CHECK(Pose_TrayPlaceTargets(3,&r,&d)==HAL_OK && fabsf(r+5)<.001f && fabsf(d-164.825f)<.001f);
    CHECK(Disc_Save() && s_disc_sequence==1U);
    CHECK(Pose_ParamsSave()); /* Preliminary layout stays independently readable. */
    CHECK(Disc_Newest(records)>=0 && s_disc_sequence==1U);
    s_disc_trim[0][1]=-11;CHECK(Disc_Save() && s_disc_sequence==2U);
    flash_fail=1;s_disc_trim[0][1]=-12;CHECK(!Disc_Save() && !flash_busy);flash_fail=0;
    memset(s_disc_trim,0,sizeof(s_disc_trim));CHECK(Disc_Load()==HAL_OK && s_disc_trim[0][1]==-11);
    s_disc_trim[0][1]=-13;CHECK(Disc_Save());
    bank=Disc_Newest(records);CHECK(bank>=0);flash[at(Disc_Address((uint8_t)bank),sizeof(records[0]))]^=1U;
    CHECK(Disc_Load()==HAL_OK && s_disc_trim[0][1]==-11); /* Torn/corrupt latest bank falls back. */
    CHECK(Pose_ParamsLoad()==HAL_OK && s_disc_trim[0][1]==-11);
    CHECK(ArmStartPose_Command("arm disc trim wait nan 0 0")==1 && s_disc_trim[0][1]==-11);
    CHECK(ArmStartPose_Command("arm disc trim wait 0 9999 0")==1 && s_disc_trim[0][1]==-11);
    chassis=1;CHECK(ArmStartPose_Command("arm disc trim wait 0 0 0")==1 && s_disc_trim[0][1]==-11);chassis=0;
    flash_busy=1;CHECK(Disc_Load()==HAL_BUSY);flash_busy=0;
    ArmStartPose_Init();CHECK(s_disc_trim[0][1]==0);
    CHECK(ArmStartPose_LoadParameters()==HAL_OK && s_disc_trim[0][1]==-11 && s_disc_flash_source==1U);
    CHECK(s_disc_trim[3][0]==2 && s_disc_trim[3][1]==-10 && s_disc_trim[3][2]==1);
    CHECK(!memcmp(&launch,&s_saved,sizeof(launch)) && linear_moves==moves);
    CHECK(ArmStartPose_Command("arm disc points")==1 && strstr(last_console,"[ARM DISC POINTS] END"));
    CHECK(ArmStartPose_Command("arm disc pose wait")==1 && s_generic && ArmStartPose_IsActive());
    CHECK(fabsf(s_run.dm_deg+1.0217f)<.001f && !memcmp(&launch,&s_saved,sizeof(launch)));
    ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;
    /* The in-window approach issues only the taught lift, never TOP/clearance. */
    grip_angle=225;enabled=1;motor.fb.enabled=1;follow_dm=1;
    axes[0].position_counts=(int64_t)(ARM_LIFT_FULL_TRAVEL_COUNTS*.45);
    axes[1].position_counts=(int64_t)(-185.0/360.0*65536.0);
    expected_reach=axes[1].position_counts;writes=lift_commands;
    CHECK(ArmStartPose_BeginMaterialApproach(-600,-185,2)==HAL_OK && s_phase==POSE_DM);
    CHECK(lift_commands==writes && !memcmp(&launch,&s_saved,sizeof(launch)));
    for(i=0;i<40 && ArmStartPose_IsActive();i++){cal_state=ARM_CAL_IDLE;step();}
    CHECK(s_phase==POSE_READY && lift_commands==writes+1U);
    CHECK(fabs(cal_lift_target-(-600.0/360.0*65536.0/ARM_LIFT_FULL_TRAVEL_COUNTS*100.0))<.001);
    CHECK(grip_angle==225 && !memcmp(&launch,&s_saved,sizeof(launch)));
    ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;
    /* A saved V238 three-pose record upgrades in RAM and never writes at boot. */
    memset(&old,0,sizeof(old));old.magic=DISC_MAGIC;old.version=1U;old.size=sizeof(old);old.sequence=41U;
    memcpy(old.trim,s_disc_trim,sizeof(old.trim));old.trim[0][1]=-8;
    old.crc=Disc_CrcBytes(&old,offsetof(disc_record_v1_t,crc));
    memset(flash,255,sizeof(flash));memcpy(flash+at(Disc_Address(0U),sizeof(old)),&old,sizeof(old));
    writes=flash_writes;ArmStartPose_Init();CHECK(Disc_Load()==HAL_OK && s_disc_sequence==41U && s_disc_trim[0][1]==-8);
    CHECK(s_disc_trim[3][0]==0 && s_disc_trim[3][1]==0 && s_disc_trim[3][2]==0 && flash_writes==writes);
    CHECK(Disc_Save() && s_disc_sequence==42U);bank=Disc_Newest(records);
    CHECK(bank==1 && records[bank].version==2U && records[bank].size==sizeof(disc_record_t));
    flash[at(Disc_Address(1U),sizeof(disc_record_t))]^=1U;
    CHECK(Disc_Load()==HAL_OK && s_disc_sequence==41U && s_disc_trim[0][1]==-8);
    return failures;
}
static int test_ui_controls(void)
{
    unsigned i;float r,d,base_r,base_d,tr,td;uint32_t moves,writes;start_profile_t launch;
    arm_ui_record_t records[2];int bank;
    memset(axes,0,sizeof(axes));memset(flash,255,sizeof(flash));
    for(i=0;i<2;i++){axes[i].travel_mm=i?178.0f:240.0f;axes[i].zero_valid=1;axes[i].position_sequence=axes[i].flags_sequence=1;axes[i].position_tick=axes[i].flags_tick=tick;axes[i].flags=0x81;}
    motor.fb.fb_count=1;motor.fb.last_tick=tick;motor.fb.vel_rads=0;
    chassis=0;cal_state=ARM_CAL_IDLE;ArmStartPose_Init();launch=s_saved;moves=linear_moves;
    CHECK(ArmStartPose_LoadParameters()==HAL_ERROR && strstr(last_console,"source=DEFAULT"));
    CHECK(!strcmp(ArmStartPose_BootParameterSource(),"DEFAULT"));
    CHECK(!flash_writes && linear_moves==moves);
    flash_busy=1;CHECK(ArmStartPose_LoadParameters()==HAL_BUSY);flash_busy=0;
    CHECK(ArmStartPose_Command("arm trim rough 1 -2 -3 0.2")==1);
    CHECK(s_ring_trim[0][0]==-2 && s_ring_trim[0][1]==-3 && fabsf(s_ring_trim[0][2]-.2f)<.001f);
    CHECK(linear_moves==moves && memcmp(&launch,&s_saved,sizeof(launch))==0);
    CHECK(Pose_RingDropBaseTargets(1,0,&base_r,&base_d)==HAL_OK);
    CHECK(Pose_RingDropTargets(1,0,&r,&d)==HAL_OK && fabsf(r-base_r+3)<.001f && fabsf(d-base_d-.2f)<.001f);
    CHECK(Pose_RingDropTargets(1,1,&tr,&td)==HAL_OK && r==tr && d==td);
    CHECK(ArmStartPose_Command("arm trim rough 1 -2 -3 0.2")==1 && s_ring_trim[0][1]==-3);
    CHECK(ArmStartPose_Command("arm trim rough 4 0 0 0")==1 && s_ring_trim[0][1]==-3);
    CHECK(ArmStartPose_Command("arm trim rough 1 nan 0 0")==1 && s_ring_trim[0][0]==-2);
    CHECK(ArmStartPose_Command("arm trim rough 1 -5000 0 0")==1 && s_ring_trim[0][0]==-2);
    CHECK(ArmStartPose_Command("arm trim rough 1 0 0 5")==1 && fabsf(s_ring_trim[0][2]-.2f)<.001f);
    CHECK(ArmStartPose_Command("arm trim temp 3 1 2 -0.3")==1 && s_ring_trim[2][0]==1);
    CHECK(Pose_RingDropTargets(3,0,&r,&d)==HAL_OK && Pose_RingDropTargets(3,1,&tr,&td)==HAL_OK && r==tr && d==td);
    CHECK(Pose_ParamsSave() && s_params_sequence==1 && flash_writes==1 && !flash_busy);
    bank=Pose_ParamsNewest(records);CHECK(bank>=0 && sizeof(arm_ui_record_t)==108U);
    CHECK(!memcmp(records[bank].trim[0],records[bank].trim[1],sizeof(s_ring_trim)));
    s_ring_trim[0][0]=-4;servo_parameters[0]=230;servo_parameters[1]=300;
    servo_parameters[2]=20;servo_parameters[3]=110;servo_parameters[4]=200;
    CHECK(Pose_ParamsSave() && s_params_sequence==2 && flash_writes==2);
    memset(s_ring_trim,0,sizeof(s_ring_trim));servo_parameters[0]=225;servo_parameters[3]=109;
    writes=flash_writes;
    CHECK(ArmStartPose_LoadParameters()==HAL_OK);
    CHECK(!strcmp(ArmStartPose_BootParameterSource(),"W25Q64"));
    CHECK(s_ring_trim[0][0]==-4 && s_ring_trim[2][0]==1 && servo_parameters[0]==230 && servo_parameters[1]==300 && servo_parameters[3]==110);
    CHECK(flash_writes==writes && linear_moves==moves);
    /* A torn save retains the previous verified bank. */
    flash_fail=1;s_ring_trim[0][0]=-6;CHECK(!Pose_ParamsSave() && !flash_busy);flash_fail=0;
    CHECK(Pose_ParamsLoad()==HAL_OK && s_ring_trim[0][0]==-4 && s_params_sequence==2);
    flash_busy=1;CHECK(!Pose_ParamsSave() && Pose_ParamsLoad()==HAL_BUSY);flash_busy=0;
    /* Legacy areas differ: use saved ROUGH as canonical, never TEMP or zero. */
    bank=Pose_ParamsNewest(records);CHECK(bank>=0);
    records[bank].trim[0][0][0]=-7;records[bank].trim[1][0][0]=-20;
    records[bank].servos[2]=205;records[bank].servos[3]=295;records[bank].servos[4]=25;
    records[bank].crc=Pose_ParamsCrc(&records[bank]);
    memcpy(flash+at(Pose_ParamsAddress((uint8_t)bank),sizeof(records[bank])),&records[bank],sizeof(records[bank]));
    CHECK(Pose_ParamsLoad()==HAL_OK && s_ring_trim[0][0]==-7 && s_ring_trim[2][0]==1);
    CHECK(servo_parameters[0]==230 && servo_parameters[1]==300 && servo_parameters[2]==205 && servo_parameters[3]==295 && servo_parameters[4]==25);
    CHECK(flash_writes==writes+1U && linear_moves==moves);
    CHECK(ArmStartPose_Command("arm trim temp 1 -1 -2 0.1")==1 && s_ring_trim[0][0]==-1);
    CHECK(Pose_ParamsSave());bank=Pose_ParamsNewest(records);CHECK(bank>=0);
    CHECK(!memcmp(records[bank].trim[0],records[bank].trim[1],sizeof(s_ring_trim)));
    CHECK(ArmStartPose_Command("arm points")==1 && strstr(last_console,"END"));
    CHECK(linear_moves==moves && memcmp(&launch,&s_saved,sizeof(launch))==0);
    memset(s_ring_trim,0,sizeof(s_ring_trim));servo_parameters[0]=225;
    CHECK(ArmStartPose_Command("arm move -20 -200 10")==1);
    CHECK(s_generic && ArmStartPose_IsActive() && memcmp(&launch,&s_saved,sizeof(launch))==0);
    ArmStartPose_Cancel();cal_state=ARM_CAL_IDLE;
    CHECK(ArmStartPose_Command("arm move nan -200 10")==1 && !ArmStartPose_IsActive());
    return failures;
}
