!+
MODULE Atmosphere1976
! ---------------------------------------------------------------------------
! PURPOSE - Compute properties of the U.S. Standard Atmosphere 1976
! AUTHORS - Steven S. Pietrobon.
!           Ralph L. Carmichael, Public Domain Aeronautical Software
!
!     REVISION HISTORY
!   DATE  VERS PERSON  STATEMENT OF CHANGES
! 28Feb95  0.1   RLC   Assembled several old codes
!  1Aug00  0.2   RLC   Copied from old Tables76
! 23Aug00  0.3   RLC   Added NitrogenNumber using QUANC8
! 24Aug00  0.4   RLC   Added KineticTemperatureGradient
! 30Aug00  0.5   RLC   Corrected numerous errors
! 30Dec00  0.6   RLC   Adapted UpperAtmosphere from Pietrobon's Unofficial
!                        Australian Standard Atmosphere
!----------------------------------------------------------------------------
IMPLICIT NONE
  CHARACTER(LEN=*),PUBLIC,PARAMETER:: ATM76_VERSION = "0.6 (12 Sept 2000)"
  real(8),PRIVATE,PARAMETER:: PI = 3.14159265
  real(8),PARAMETER:: REARTH = 6356.766               ! radius of the Earth (km)
  real(8),PARAMETER:: GMR = 34.163195                             ! gas constant
  real(8),PARAMETER:: GZERO = 9.80665 !  accel. of gravity, m/sec^2

!  real(8),PARAMETER:: FT2METERS = 0.3048       ! mult. ft. to get meters (exact)
!  real(8),PARAMETER:: KELVIN2RANKINE = 1.8             ! mult K to get deg R
!  real(8),PARAMETER:: PSF2NSM = 47.880258          ! mult lb/sq.ft to get N/sq.m
!  real(8),PARAMETER:: SCF2KCM = 515.379         ! mult slug/cu.ft to get kg/cu.m
  real(8),PARAMETER:: TZERO = 288.15                ! temperature at sealevel, K
  real(8),PARAMETER:: PZERO = 101325.0            ! pressure at sealevel, N/sq.m
  real(8),PARAMETER:: RHOZERO = 1.2250            ! density at sealevel, kg/cu.m
  real(8),PARAMETER:: RSTAR = 8314.32       ! perfect gas constant, N-m/(kmol-K)
  real(8),PARAMETER:: ASOUNDZERO = 340.294   ! speed of sound at sealevel, m/sec

  real(8),PARAMETER:: BETAVISC = 1.458E-6    ! viscosity term, N s/(sq.m sqrt(K)
  real(8),PARAMETER:: SUTH = 110.4                    ! Sutherland's constant, K


  real(8),PARAMETER:: MZERO      = 28.9644 ! molecular weight of air at sealevel
!  real(8),PARAMETER:: AVOGADRO =  6.022169E26        ! 1/kmol, Avogadro constant
!  real(8),PARAMETER:: BOLTZMANN = 1.380622E-23        ! Nm/K, Boltzmann constant


! TABLE 5 - DEFINITION OF KINETIC TEMPERATURE FROM 86 km to 1000 km
  real(8),PARAMETER:: Z7 =  86.0,  T7=186.8673
  real(8),PARAMETER:: z8 =  91.0,  T8=T7
  real(8),PARAMETER:: Z9 = 110.0,  T9=240.0
  real(8),PARAMETER:: Z10= 120.0, T10=360.0
!  real(8),PARAMETER:: Z11= 500.0, T11=999.2356   ! not used
  real(8),PARAMETER:: Z12=1000.0, T12=1000.0


  real(8),PARAMETER:: FT2METERS = 0.3048               ! mult. ft. to get meters (exact)
  real(8),PARAMETER:: KELVIN2RANKINE = 1.8             ! mult deg K to get deg R
  real(8),PARAMETER:: PSF2NSM = 47.880258          ! mult lb/sq.ft to get N/sq.m
  real(8),PARAMETER:: SCF2KCM = 515.379         ! mult slug/cu.ft to get kg/cu.m




CONTAINS

!+
FUNCTION EvaluateCubic(a,fa,fpa, b,fb,fpb, u) RESULT(fu)
! ---------------------------------------------------------------------------
! PURPOSE - Evaluate a cubic polynomial defined by the function and the
!   1st derivative at two points
  real(8),INTENT(IN):: u   ! point where function is to be evaluated
  real(8),INTENT(IN):: a,fa,fpa   ! a, f(a), f'(a)  at first point
  real(8),INTENT(IN):: b,fb,fpb   ! b, f(b), f'(b)  at second point
  real(8):: fu                    ! computed value of f(u)

  real(8):: d,t,p
!----------------------------------------------------------------------------
  d=(fb-fa)/(b-a)
  t=(u-a)/(b-a)
  p=1.0-t

  fu = p*fa + t*fb - p*t*(b-a)*(p*(d-fpa)-t*(d-fpb))
  RETURN
END Function EvaluateCubic   ! ----------------------------------------------

!+
FUNCTION KineticTemperature(z) RESULT(t)
!   -------------------------------------------------------------------------
! PURPOSE - Compute kinetic temperature above 86 km.

  real(8),INTENT(IN)::  z     ! geometric altitude, km.                        
  real(8):: t     ! kinetic temperature, K

  real(8),PARAMETER:: C1 = -76.3232  ! uppercase A in document
  real(8),PARAMETER:: C2 = 19.9429   ! lowercase a in document
  real(8),PARAMETER:: C3 = 12.0
  real(8),PARAMETER:: C4 = 0.01875   ! lambda in document
  real(8),PARAMETER:: TC = 263.1905

  real(8):: xx,yy
!----------------------------------------------------------------------------
  IF (z <= Z8) THEN
    t=T7
  ELSE IF (z < Z9) THEN  
    xx=(z-Z8)/C2                        ! from Appendix B, p.223
    yy=SQRT(1.0-xx*xx)
    t=TC+C1*yy
  ELSE IF (z <= Z10) THEN
    t=T9+C3*(z-Z9)
  ELSE
    xx=(REARTH+Z10)/(REARTH+z)
    yy=(T12-T10)*EXP(-C4*(z-Z10)*xx)
    t=T12-yy
  END IF

  RETURN
END Function KineticTemperature   ! -----------------------------------------


!+
SUBROUTINE UpperAtmosphere(alt, sigma, delta, theta)
!   -------------------------------------------------------------------------
! PURPOSE - Compute the properties of the 1976 standard atmosphere from
!   86 km. to 1000 km.

  IMPLICIT NONE
!============================================================================
!     A R G U M E N T S                                                     |
!============================================================================
  real(8),INTENT(IN)::  alt    ! geometric altitude, km.                        
!  real(8),INTENT(INOUT)
real(8):: sigma  ! density/sea-level standard density              
!  real(8),INTENT(INOUT)
real(8):: delta  ! pressure/sea-level standard pressure           
!  real(8),INTENT(INOUT)
real(8):: theta  ! temperature/sea-level standard temperature
!============================================================================
!     L O C A L   C O N S T A N T S                                         |
!============================================================================

! altitude table (m)
  real(8),PARAMETER,DIMENSION(23):: Z = (/      &
     86.,  93., 100., 107., 114., &
    121., 128., 135., 142., 150., &
    160., 170., 180., 190., 200., &
    250., 300., 400., &
    500., 600., 700., 800., 1000. /)

! pressure table  (Pa)
!  real(8),PARAMETER,DIMENSION(SIZE(Z)):: P = (/                                &
!    3.7338e-1, 1.0801e-1, 3.2011e-2, 1.0751E-2, 4.4473e-3, &
!    2.3402e-3, 1.4183e-3, 9.3572e-4, 6.5297e-4, 4.5422e-4, &
!    3.0397e-4, 2.1212e-4, 1.5273e-4, 1.1267e-4, 8.4743e-5, &
!    2.4767e-5, 8.7704e-6, 1.4518e-6, &
!    3.0236e-7, 8.2130e-8, 3.1908e-8, 1.7036e-8, 7.5138e-9 /)

! density table  kg/m**3
!  real(8),PARAMETER,DIMENSION(SIZE(Z)):: RHO = (/                              &
!    6.9579e-06, 1.9997e-06, 5.6041e-07, 1.6426E-07, 4.9752e-08, &
!    1.9768e-08, 9.7173e-09, 5.4652e-09, 3.3580e-09, 2.0757e-09, &
!    1.2332e-09, 7.8155e-10, 5.1944e-10, 3.5808e-10, 2.5409e-10, &
!    6.0732e-11, 1.9160e-11, 2.8031e-12, &
!    5.2159e-13, 1.1369e-13, 3.0698e-14, 1.1361e-14, 3.5614e-15 /)


!fit2000
  real(8),PARAMETER,DIMENSION(SIZE(Z)):: LOGP = (/               &
  -0.985159,  -2.225531,  -3.441676,  -4.532756,  -5.415458,  &
  -6.057519,  -6.558296,  -6.974194,  -7.333980,  -7.696929,  &
  -8.098581,  -8.458359,  -8.786839,  -9.091047,  -9.375888,  &
 -10.605998, -11.644128, -13.442706, -15.011647, -16.314962,  &
 -17.260408, -17.887938, -18.706524 /)

 real(8),PARAMETER,DIMENSION(SIZE(Z)):: DLOGPDZ = (/             &
 -11.875633, -13.122514, -14.394597, -15.621816, -16.816216,  &
 -17.739201, -18.449358, -19.024864, -19.511921, -19.992968,  &
 -20.513653, -20.969742, -21.378269, -21.750265, -22.093332,  &
 -23.524549, -24.678196, -26.600296, -28.281895, -29.805302,  &
 -31.114578, -32.108589, -33.268623 /)

  real(8),PARAMETER,DIMENSION(SIZE(Z)):: LOGRHO = (/             &
  -0.177700,  -0.176950,  -0.167294,  -0.142686,  -0.107868,  &
  -0.079313,  -0.064668,  -0.054876,  -0.048264,  -0.042767,  &
  -0.037847,  -0.034273,  -0.031539,  -0.029378,  -0.027663,  &
  -0.022218,  -0.019561,  -0.016734,  -0.014530,  -0.011315,  &
  -0.007673,  -0.005181,  -0.003500 /)

  real(8),PARAMETER,DIMENSION(SIZE(Z)):: DLOGRHODZ = (/          & 
  -0.177900,  -0.180782,  -0.178528,  -0.176236,  -0.154366,  &
  -0.113750,  -0.090551,  -0.075044,  -0.064657,  -0.056087,  &
  -0.048485,  -0.043005,  -0.038879,  -0.035637,  -0.033094,  &
  -0.025162,  -0.021349,  -0.017682,  -0.016035,  -0.014330,  &
  -0.011626,  -0.008265,  -0.004200 /)


!============================================================================
!     L O C A L   V A R I A B L E S                                         |
!============================================================================
  INTEGER:: i,j,k                                                  ! counters
!  real(8):: h                                       ! geopotential altitude (km)
!  real(8):: tgrad, tbase      ! temperature gradient and base temp of this layer
!  real(8):: tlocal                                           ! local temperature
!  real(8):: deltah                             ! height above base of this layer

  real(8):: p,rho
!----------------------------------------------------------------------------

  IF (alt >= Z(SIZE(Z))) THEN          ! trap altitudes greater than 1000 km.
    delta=1E-20
    sigma=1E-21
    theta=1000.0/TZERO
    RETURN
  END IF

  i=1 
  j=SIZE(Z)                                    ! setting up for binary search
  DO
    k=(i+j)/2                                              ! integer division
    IF (alt < Z(k)) THEN
      j=k
    ELSE
      i=k
    END IF   
    IF (j <= i+1) EXIT
  END DO

  p=EXP(EvaluateCubic(Z(i),LOGP(i),DLOGPDZ(i), &
                      Z(i+1),LOGP(i+1),DLOGPDZ(i+1), alt))
  delta=p/PZERO

  rho=EXP(EvaluateCubic(Z(i),LOGRHO(i),DLOGRHODZ(i), &
                      Z(i+1),LOGRHO(i+1),DLOGRHODZ(i+1), alt))
  sigma=rho/RHOZERO

  theta=KineticTemperature(alt)/TZERO
  RETURN
END Subroutine UpperAtmosphere   ! ------------------------------------------

!+
SUBROUTINE LowerAtmosphere(alt, sigma, delta, theta)
!   -------------------------------------------------------------------------
! PURPOSE - Compute the properties of the 1976 standard atmosphere to 86 km.

  IMPLICIT NONE
!============================================================================
!     A R G U M E N T S                                                     |
!============================================================================
  real(8),INTENT(IN)::  alt    ! geometric altitude, km.                        
  real(8),INTENT(INOUT):: sigma  ! density/sea-level standard density              
  real(8),INTENT(INOUT):: delta  ! pressure/sea-level standard pressure           
  real(8),INTENT(INOUT):: theta  ! temperature/sea-level standard temperature
!============================================================================
!     L O C A L   C O N S T A N T S                                         |
!============================================================================
  real(8),PARAMETER:: REARTH = 6369.0                 ! radius of the Earth (km)
  real(8),PARAMETER:: GMR = 34.163195                             ! gas constant
  INTEGER,PARAMETER:: NTAB=8       ! number of entries in the defining tables
!============================================================================
!     L O C A L   V A R I A B L E S                                         |
!============================================================================
  INTEGER:: i,j,k                                                  ! counters
  real(8):: h                                       ! geopotential altitude (km)
  real(8):: tgrad, tbase      ! temperature gradient and base temp of this layer
  real(8):: tlocal                                           ! local temperature
  real(8):: deltah                             ! height above base of this layer
!============================================================================
!     L O C A L   A R R A Y S   ( 1 9 7 6   S T D.  A T M O S P H E R E )   |
!============================================================================
  real(8),DIMENSION(NTAB),PARAMETER:: htab= &
                          (/0.0, 11.0, 20.0, 32.0, 47.0, 51.0, 71.0, 84.852/)
  real(8),DIMENSION(NTAB),PARAMETER:: ttab= &
          (/288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65, 186.946/)
  real(8),DIMENSION(NTAB),PARAMETER:: ptab= &
               (/1.0, 2.233611E-1, 5.403295E-2, 8.5666784E-3, 1.0945601E-3, &
                                     6.6063531E-4, 3.9046834E-5, 3.68501E-6/)
  real(8),DIMENSION(NTAB),PARAMETER:: gtab= &
                                (/-6.5, 0.0, 1.0, 2.8, 0.0, -2.8, -2.0, 0.0/)
!----------------------------------------------------------------------------
  h=alt*REARTH/(alt+REARTH)      ! convert geometric to geopotential altitude

  i=1 
  j=NTAB                                  ! setting up for binary search
  DO
    k=(i+j)/2                                              ! integer division
    IF (h < htab(k)) THEN
      j=k
    ELSE
      i=k
    END IF   
    IF (j <= i+1) EXIT
  END DO

  tgrad=gtab(i)                                     ! i will be in 1...NTAB-1
  tbase=ttab(i)
  deltah=h-htab(i)
  tlocal=tbase+tgrad*deltah
  theta=tlocal/ttab(1)                                    ! temperature ratio

  IF (tgrad == 0.0) THEN                                     ! pressure ratio
    delta=ptab(i)*EXP(-GMR*deltah/tbase)
  ELSE
    delta=ptab(i)*(tbase/tlocal)**(GMR/tgrad)
  END IF

  sigma=delta/theta                                           ! density ratio
  RETURN
END Subroutine LowerAtmosphere   ! ------------------------------------------

!+
SUBROUTINE SimpleAtmosphere(alt,sigma,delta,theta)
!   -------------------------------------------------------------------------
! PURPOSE - Compute the characteristics of the atmosphere below 20 km.

! NOTES-Correct to 20 km. Only approximate above there

  IMPLICIT NONE
!============================================================================
!     A R G U M E N T S                                                     |
!============================================================================
  real(8),INTENT(IN)::  alt    ! geometric altitude, km.
  real(8),INTENT(OUT):: sigma  ! density/sea-level standard density             
  real(8),INTENT(OUT):: delta  ! pressure/sea-level standard pressure            
  real(8),INTENT(OUT):: theta  ! temperature/sea-level standard temperature   
!============================================================================
!     L O C A L   C O N S T A N T S                                         |
!============================================================================
  real(8),PARAMETER:: REARTH = 6369.0                ! radius of the Earth (km)
  real(8),PARAMETER:: GMR = 34.163195                            ! gas constant
!============================================================================
!     L O C A L   V A R I A B L E S                                         |
!============================================================================
  real(8):: h   ! geopotential altitude
!----------------------------------------------------------------------------
  h=alt*REARTH/(alt+REARTH)      ! convert geometric to geopotential altitude

  IF (h < 11.0) THEN
    theta=1.0+(-6.5/288.15)*h                                   ! Troposphere
    delta=theta**(GMR/6.5)
  ELSE
    theta=216.65/288.15                                        ! Stratosphere
    delta=0.2233611*EXP(-GMR*(h-11.0)/216.65)
  END IF

  sigma=delta/theta
  RETURN
END Subroutine SimpleAtmosphere   ! -----------------------------------------

!+
FUNCTION Viscosity(theta) RESULT(visc)
!   -------------------------------------------------------------------------
! PURPOSE - Compute viscosity using Sutherland's formula.
!        Returns viscosity in kg/(meter-sec)

  IMPLICIT NONE
  real(8),INTENT(IN) :: theta                ! temperature/sea-level temperature  
  real(8):: visc
  real(8):: temp                              ! temperature in deg Kelvin
!----------------------------------------------------------------------------
  temp=TZERO*theta
  visc=BETAVISC*Sqrt(temp*temp*temp)/(temp+SUTH)
  RETURN
END Function Viscosity   ! --------------------------------------------

!+
SUBROUTINE Atmosphere(alt,sigma,delta,theta)
!   -------------------------------------------------------------------------
! PURPOSE - Compute the characteristics of the U.S. Standard Atmosphere 1976

  IMPLICIT NONE
  real(8),INTENT(IN)::  alt    ! geometric altitude, km.
!  real(8),INTENT(OUT)
real(8):: sigma  ! density/sea-level standard density             
!  real(8),INTENT(OUT)
real(8):: delta  ! pressure/sea-level standard pressure            
!  real(8),INTENT(OUT)
real(8):: theta  ! temperature/sea-level standard temperature   
!============================================================================
  IF (alt > 86.0) THEN
    CALL UpperAtmosphere(alt,sigma,delta,theta)
  ELSE
    CALL LowerAtmosphere(alt,sigma,delta,theta)
  END IF
  RETURN
END Subroutine Atmosphere   ! -----------------------------------------------

END Module Atmosphere1976   ! ===============================================

