program Post
implicit none

integer :: i,eof
real(8),parameter :: PI = 3.1415926535898d0

!- Switch
integer :: access
integer :: sw_log_ballistic,sw_log_decent
integer :: sw_table_landingrange,sw_table_altitude,sw_table_mach,sw_table_vamax,sw_table_vatop
integer :: sw_table_timetop,sw_table_timehard,sw_table_timesoft,sw_table_timesecond
integer :: sw_table_vlc,sw_table_acclc

!- Log
real(8) :: t,Pa,Ta,rho,g,Cs,Vw(2),mf,mox,mp,m,lcgox,lcgp,lcg,Dx,F(3),Acce(3),Va(3),Va_abs,Mach,Ve(3)
real(8) :: theta,psi,fai,alpha,beta,Position(3)

!- Table
real(8) :: LandingRange(56,4)
real(8) :: temp78(7,8)

!--------------------------------------
!-        Switch File Exist
!--------------------------------------
!- status == 0 : exist , status != 0 : not exist
sw_log_ballistic = access('OutputLog/BallisticLog.dat',' ')
sw_log_decent = access('OutputLog/DecentLog.dat',' ')


sw_table_landingrange = access('OutputTable/Landing_Range_v2.dat',' ')
sw_table_altitude = access('OutputTable/Max_Altitude.dat',' ')
sw_table_mach = access('OutputTable/Mach_max.dat',' ')
sw_table_vamax = access('OutputTable/Va_max.dat',' ')
sw_table_vatop = access('OutputTable/Va_Top.dat',' ')
sw_table_timetop = access('OutputTable/time_top.dat',' ')
sw_table_timehard = access('OutputTable/time_Hard_Landing.dat',' ')
sw_table_timesoft = access('OutputTable/time_Para_Landing.dat',' ')
sw_table_timesecond = access('OutputTable/time_Second_Para.dat',' ')
sw_table_vlc = access('OutputTable/LaunchClear_Velocity.dat',' ')
sw_table_acclc = access('OutputTable/LaunchClear_Acceleration.dat',' ')


!--------------------------------------
!-             Log Read
!--------------------------------------



if (sw_log_ballistic == 0) then
  eof = 0
  print *, "Converting... : Log"
  open (100,file='OutputLog/BallisticLog.dat',status="old",form="unformatted")
  open (101,file='OutputLog/Position_log.csv',status="replace")
  open (102,file='OutputLog/Attitude_log.csv',status="replace")
  open (103,file='OutputLog/Velocity_log.csv',status="replace")
  open (104,file='OutputLog/Acceleration_log.csv',status="replace")
  open (105,file='OutputLog/Force_log.csv',status="replace")
  open (106,file='OutputLog/Structure_log.csv',status="replace")
  open (107,file='OutputLog/Atmosphere_log.csv',status="replace")
  
  do while(eof >= 0)
    read (100,iostat = eof) t,Pa,Ta,rho,g,Cs,Vw(1),Vw(2),mf,mox,mp,m,lcgox,lcgp,lcg,Dx,F(1),F(2),F(3),Acce(1),Acce(2),Acce(3), &
                          & Va(1),Va(2),Va(3),Va_abs,Mach,Ve(1),Ve(2),Ve(3),theta,psi,fai,alpha,beta, &
                          & Position(1),Position(2),Position(3)
    write (101,'(4(f0.6,a))') t,',',Position(1),',',Position(2),',',Position(3)
    write (102,'(6(f0.6,a))') t,',',theta,',',psi,',',fai,',',alpha,',',beta
    write (103,'(9(f0.6,a))') t,',',Ve(1),',',Ve(2),',',Ve(3),',',Va(1),',',Va(2),',',Va(3),',',Va_abs,',',Mach
    write (104,'(4(f0.6,a))') t,',',Acce(1),',',Acce(2),',',Acce(3)
    write (105,'(5(f0.6,a))') t,',',Dx,',',F(1),',',F(2),',',F(3)
    write (106,'(8(f0.6,a))') t,',',mf,',',mox,',',mp,',',m,',',lcgox,',',lcgp,',',lcg
    write (107,'(8(f0.6,a))') t,',',Pa,',',Ta,',',rho,',',g,',',Cs,',',Vw(1),',',Vw(2)
    
  end do
  close(100)
  close(101) ; close(102) ; close(103) ; close(104) ; close(105) ; close(106) ; close(107)
end if


!--------------------------------------
!-           Table Read
!--------------------------------------

if (sw_table_landingrange == 0) then
  print *, "Converting... : LandingRange"
  open (200,file='OutputTable/Landing_Range_v2.dat',status="old",form="unformatted")
  read(200) LandingRange
  close(200)
  open (201,file='OutputTable/Landing_Range_v2.csv',status="replace")
  do i = 1,56
    write (201,'(4(f0.3,a))') LandingRange(i,1),',',LandingRange(i,2),',',LandingRange(i,3),',',LandingRange(i,4)
  end do
  close(201)
end if
if (sw_table_altitude == 0) then
  print *, "Converting... : Altitude Table"
  open (202,file='OutputTable/Max_Altitude.dat',status="old",form="unformatted")
  read(202) temp78
  close(202)
  open (203,file='OutputTable/Max_Altitude.csv',status="replace")
  do i = 1,7
    write (203,'(8(f0.3,a))') temp78(i,1),',',temp78(i,2),',',temp78(i,3),',',temp78(i,4),',', &
                            & temp78(i,5),',',temp78(i,6),',',temp78(i,7),',',temp78(i,8)
  end do
  close(203)
end if
if (sw_table_mach == 0) then
  print *, "Converting... : Mach Table"
  open (204,file='OutputTable/Mach_max.dat',status="old",form="unformatted")
  read(204) temp78
  close(204)
  open (205,file='OutputTable/Mach_max.csv',status="replace")
  do i = 1,7
    write (205,'(8(f0.3,a))') temp78(i,1),',',temp78(i,2),',',temp78(i,3),',',temp78(i,4),',', &
                            & temp78(i,5),',',temp78(i,6),',',temp78(i,7),',',temp78(i,8)
  end do
  close(205)
end if
if (sw_table_vamax == 0) then
  print *, "Converting... : Vamax Table"
  open (206,file='OutputTable/Va_max.dat',status="old",form="unformatted")
  read(206) temp78
  close(206)
  open (207,file='OutputTable/Va_max.csv',status="replace")
  do i = 1,7
    write (207,'(8(f0.3,a))') temp78(i,1),',',temp78(i,2),',',temp78(i,3),',',temp78(i,4),',', &
                            & temp78(i,5),',',temp78(i,6),',',temp78(i,7),',',temp78(i,8)
  end do
  close(207)
end if
if (sw_table_vatop == 0) then
  print *, "Converting... : Vatop Table"
  open (208,file='OutputTable/Va_Top.dat',status="old",form="unformatted")
  read(208) temp78
  close(208)
  open (209,file='OutputTable/Va_Top.csv',status="replace")
  do i = 1,7
    write (209,'(8(f0.3,a))') temp78(i,1),',',temp78(i,2),',',temp78(i,3),',',temp78(i,4),',', &
                            & temp78(i,5),',',temp78(i,6),',',temp78(i,7),',',temp78(i,8)
  end do
  close(209)
end if
if (sw_table_timetop == 0) then
  print *, "Converting... : ttop Table"
  open (210,file='OutputTable/time_top.dat',status="old",form="unformatted")
  read(210) temp78
  close(210)
  open (211,file='OutputTable/time_top.csv',status="replace")
  do i = 1,7
    write (211,'(8(f0.3,a))') temp78(i,1),',',temp78(i,2),',',temp78(i,3),',',temp78(i,4),',', &
                            & temp78(i,5),',',temp78(i,6),',',temp78(i,7),',',temp78(i,8)
  end do
  close(211)
end if
if (sw_table_timehard == 0) then
  print *, "Converting... : thard Table"
  open (212,file='OutputTable/time_Hard_Landing.dat',status="old",form="unformatted")
  read(212) temp78
  close(212)
  open (213,file='OutputTable/time_Hard_Landing.csv',status="replace")
  do i = 1,7
    write (213,'(8(f0.3,a))') temp78(i,1),',',temp78(i,2),',',temp78(i,3),',',temp78(i,4),',', &
                            & temp78(i,5),',',temp78(i,6),',',temp78(i,7),',',temp78(i,8)
  end do
  close(213)
end if
if (sw_table_timesoft == 0) then
  print *, "Converting... : tsoft Table"
  open (214,file='OutputTable/time_Para_Landing.dat',status="old",form="unformatted")
  read(214) temp78
  close(214)
  open (215,file='OutputTable/time_Para_Landing.csv',status="replace")
  do i = 1,7
    write (215,'(8(f0.3,a))') temp78(i,1),',',temp78(i,2),',',temp78(i,3),',',temp78(i,4),',', &
                            & temp78(i,5),',',temp78(i,6),',',temp78(i,7),',',temp78(i,8)
  end do
  close(215)
end if
if (sw_table_timesecond == 0) then
  print *, "Converting... : tTSSS Table"
  open (216,file='OutputTable/time_Second_Para.dat',status="old",form="unformatted")
  read(216) temp78
  close(216)
  open (217,file='OutputTable/time_Second_Para.csv',status="replace")
  do i = 1,7
    write (217,'(8(f0.3,a))') temp78(i,1),',',temp78(i,2),',',temp78(i,3),',',temp78(i,4),',', &
                            & temp78(i,5),',',temp78(i,6),',',temp78(i,7),',',temp78(i,8)
  end do
  close(217)
end if
if (sw_table_vlc == 0) then
  print *, "Converting... : Vlc Table"
  open (218,file='OutputTable/LaunchClear_Velocity.dat',status="old",form="unformatted")
  read(218) temp78
  close(218)
  open (219,file='OutputTable/LaunchClear_Velocity.csv',status="replace")
  do i = 1,7
    write (219,'(8(f0.3,a))') temp78(i,1),',',temp78(i,2),',',temp78(i,3),',',temp78(i,4),',', &
                            & temp78(i,5),',',temp78(i,6),',',temp78(i,7),',',temp78(i,8)
  end do
  close(219)
end if
if (sw_table_acclc == 0) then
  print *, "Converting... : Acclc Table"
  open (220,file='OutputTable/LaunchClear_Acceleration.dat',status="old",form="unformatted")
  read(220) temp78
  close(220)
  open (221,file='OutputTable/LaunchClear_Acceleration.csv',status="replace")
  do i = 1,7
    write (221,'(8(f0.3,a))') temp78(i,1),',',temp78(i,2),',',temp78(i,3),',',temp78(i,4),',', &
                            & temp78(i,5),',',temp78(i,6),',',temp78(i,7),',',temp78(i,8)
  end do
  close(221)
end if

print *, "Complete Convert..."
print *, "Please Next Post Process..."

contains

!**********************************************************************
!                       Universal Subroutine
!**********************************************************************
function deg2rad (x)
  real(8) :: deg2rad,x
  deg2rad = x*PI / 180.0d0
end function

function rad2deg (x)
  real(8) :: rad2deg,x
  rad2deg = x*180.0d0 / PI
end function

end program Post
