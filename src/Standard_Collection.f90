module Standard_Collection
use Rocket_Class
implicit none

real(8),parameter :: PI = 3.1415926535898d0 , R = 287.1d0 , gamma = 1.4d0
real(8),parameter :: Re = 6356766.0d0 !---地球半径--
real(8),parameter :: GT = 0.0065d0 !-------気温減衰率K/m--------

!**********************************************************************
!                        Program Variable
!**********************************************************************
real :: t1,t2                 ! 計算時間測定用
real(8) :: t
real(8) :: dt
real(8) :: freq               ! 推力履歴のサンプリング周波数[Hz]
integer :: i,j
integer :: Flight_Status = 0  ! 飛翔フェーズ
integer :: index_wind,index_wangle
integer :: index_burn         ! 推力履歴のサイズ
integer :: eof = 0            ! 推力履歴読み込み用

! Mach-Cdスプライン用
real(8),allocatable :: Mach_base(:),Cd_base(:)
integer :: n_base = 0

type(Rocket_Type) :: Rocket

!------------------------------------
!-            Condition
!------------------------------------
real(8) :: Vw_abs,Vw_angle        ! 風速,風向
real(8) :: Vw(3)                  ! (xe,ye,ze)の風速
real(8) :: Hw,Wh                  ! 風速測定高度,高度係数
real(8) :: Pa,Ta,rho,g,Cs         ! Air Pressure,Air Temperature,Air Dencity,Gravity,Speed of Sonic 
real(8) :: Pa_0,Ta_0,rho_0,g0
real(8) :: LL                     ! ランチャレール長[m]

!------------------------------------
!-              Save
!------------------------------------
real(8) :: t_top,Position_top(3),Va_top,Ve_top(3),m_top
real(8) :: V_lc,Acc_lc,Va_max,Mach_max
real(8) :: t_sepa2

!**********************************************************************
!                        Switch Variable
!**********************************************************************
integer :: LogTable_sw ! 0:Log/1:Table
real(8) :: Vw_min,Vw_max,Vw_delta
real(8) :: Vw_angle_min,Vw_angle_max,Vw_angle_delta
integer :: Decent_sw ! 0:Ballistic/1:Decent

real(8) :: LandingRange(56,4)
real(8),dimension(7,8) :: AltitudeTable,MachTable,VaMaxTable,VaApogeeTable
real(8),dimension(7,8) :: TimeApogeeTable,TimeHardTable,TimeDecentTable,TimeTSSSTable

contains

!**********************************************************************
!                       Personal Subroutine
!**********************************************************************
!------------------------------------
!-           Mach-Cd Read
!-使う前にsplineの最後の引数を0で呼び出し
!-save属性でスプラインに必要な変数が保持される
!-
!------------------------------------

subroutine setMachCd ()
  real(8) :: Mach,Cd
  open (103,file='Mach_Cd.dat',status='old')
  read (103,*) !１行目のスキップ
  do
    read (103,'()',end=80)
    n_base = n_base + 1
  end do
  80 close(103)
  allocate(Mach_base(n_base),Cd_base(n_base))
  open (103,file='Mach_Cd.dat',status='old')
  read (103,*) !1行目のスキップ
  do i = 1,n_base
    read (103,*) Mach_base(i),Cd_base(i)
  end do
  close(103)
  call spline(Mach_base,Cd_base,n_base-1,Mach,Cd,0)
end subroutine setMachCd

function getMachCd (Mach)
  real(8) :: getMachCd
  real(8),intent(in) :: Mach
  real(8) :: Cd
  call spline(Mach_base,Cd_base,n_base-1,Mach,Cd,1)
  getMachCd = Cd
end function getMachCd


!------------------------------------
!-           Thrust Read
!-推力履歴は必ず作動開始からのデータであること
!-質量減少はthrustがあれば発生するようになっている
!------------------------------------

subroutine setThrust(Rocket)
  type(Rocket_Type),intent(inout) :: Rocket
  real(8) :: temp
  real(8),allocatable :: temp_array(:)
  allocate(Rocket%thrust(1))
  Rocket%thrust = 0.0d0
  open (62,file='thrust.dat',status='old')
  i = 1
  do while(eof >= 0)
    read (62,*,iostat = eof) temp
    
    allocate(temp_array(i))
    !temp_array = Rocket%thrust
    do j = 1,i
      temp_array(j) = Rocket%thrust(j)
    end do
    deallocate(Rocket%thrust)
    allocate(Rocket%thrust(i+1))
    do j = 1,i
      Rocket%thrust(j) = temp_array(j)
    end do
    Rocket%thrust(i+1) = temp
    deallocate(temp_array)
    i = i + 1
  end do
  index_burn = i - 1 
  close(62)
end subroutine setThrust


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

function deg2kelvin (x)
  real(8) :: deg2kelvin,x
  deg2kelvin = x + 273.15
end function

!------------------------------------
!-           符号チェック
!-第2引数が1か-1で符号第1引数の符号チェック
!-第2引数に合わせた符号になる
!-射角や減衰モーメント係数のエラーチェック
!------------------------------------

subroutine sign_check(x,sign)
  real(8),intent(inout) :: x
  integer,intent(in) :: sign
  
  select case (sign)
    case(1:)
      if (x < 0.0d0) then
        x = -1.0d0 * x
      end if
    case(:-1)
      if (x > 0.0d0) then
        x = -1.0d0 * x
      end if
  end select
end subroutine sign_check

!------------------------------------
!-      配列の最大要素index取得
!-quatarnionの生成時に最大要素のindexが必要
!-
!------------------------------------

subroutine maxcheck(array,max)
  real(8),intent(in) :: array(:)
  integer,intent(out) :: max
  integer :: i,arraysize
  real(8) :: max_temp
  
  arraysize = size(array)
  max_temp = 0.0d0
  
  do i = 1,arraysize
    if (array(i) > max_temp) then
      max_temp = array(i)
      max = i
    end if
  end do
end subroutine maxcheck

!------------------------------------
!-        配列への要素追加
!-pythonのappendに近いもの
!-
!------------------------------------

subroutine append(toarray,fromarray)
  real(8),allocatable,intent(inout) :: toarray(:,:)
  real(8),intent(in) :: fromarray(:)
  real(8),allocatable :: temp(:,:)
  integer line_size,row_size ! 行,列
  integer :: i
    
  if (.not. allocated(toarray)) then
    row_size = size(fromarray)
    allocate(toarray(1,row_size))
    toarray(1,:) = fromarray
  else
    if (size(toarray(1,:)) == size(fromarray)) then
      line_size = size(toarray(:,1))
      row_size = size(toarray(1,:))
      allocate(temp(line_size,row_size))
      temp = toarray
      deallocate(toarray)
      allocate(toarray(line_size+1,row_size))
      do i = 1,line_size
        toarray(i,:) = temp(i,:)
      end do
      toarray(line_size+1,:) = fromarray
    else
      print *,"Error : Array Size"
    end if
  end if
end subroutine append

function Abs_Vector(Vector)
  real(8),intent(in) :: Vector(3)
  real(8) :: Abs_Vector
  Abs_Vector = sqrt(Vector(1) * Vector(1) + Vector(2) * Vector(2) + Vector(3) * Vector(3))
end function Abs_Vector

function Integral_3D(x,x_pre,dt)
  real(8) :: Integral_3D(3)
  real(8),intent(in) :: x(:),x_pre(:),dt
  
  Integral_3D(1) = 0.5d0*(x(1) + x_pre(1))*dt
  Integral_3D(2) = 0.5d0*(x(2) + x_pre(2))*dt
  Integral_3D(3) = 0.5d0*(x(3) + x_pre(3))*dt
end function Integral_3D

!-----------------------------------------------------------------------------!
!Construct cubic spline interpolation from a given data set yp(xp) 
!yp(i): given data values 
!xp(i): given points
!     where i=0,1,2,...nd 
!     np: maximum number of data point (total number of points is np+1)
!
!
!y: interpolated value
!x: interpolation location 
!
!mode == 0: initial setting
!mode == 1: spline interpolation
!-----------------------------------------------------------------------------!
subroutine spline(xp,yp,np,x,y,mode)
  integer,intent(in) :: np !data number
  integer :: mode,i,i1,i2
  real(8) :: x,y
  real(8),intent(in) :: xp(0:np),yp(0:np)
  real(8),allocatable :: h(:),a_mat(:),b_mat(:),c_mat(:),d_mat(:)
  real(8),allocatable,save :: a(:),b(:),c(:)!,x_save(:),y_save(;)
  
  ! Coefficient determination
  ! mode == 0
  if (mode == 0) then
    allocate(h(0:np),a_mat(0:np),b_mat(0:np),c_mat(0:np),d_mat(0:np))
    allocate(a(0:np-1),b(0:np-1),c(0:np))
    
    do i = 0,np-1
      h(i) = xp(i+1) - xp(i)
    end do
    h(np) = xp(np) - xp(np-1)
    
    b_mat(0) = 2.0d0*h(0)
    c_mat(0) = h(0)
    d_mat(0) = 3.0d0*(yp(1) - yp(0))
    do i = 1,np-1
      a_mat(i) = h(i)
      b_mat(i) = 2.0d0*(xp(i+1) - xp(i-1))
      c_mat(i) = h(i-1)
      d_mat(i) = (3.0d0*(yp(i) - yp(i-1))*h(i) / h(i-1)) + (3.0d0*(yp(i+1) - yp(i))*h(i-1) / h(i))
    end do
    a_mat(np) = h(np)
    b_mat(np) = 2.0d0*(h(np))
    d_mat(np) = 3.0d0*(yp(np) - yp(np-1))
    
    call tdma(a_mat,b_mat,c_mat,d_mat,np+1,c)
    
    do i = 0,np-1
      b(i) = (3.0d0*(yp(i+1) - yp(i)) - h(i)*(c(i+1) + 2.0d0*c(i))) / h(i)**2
      a(i) = (c(i+1) - c(i) - 2.0d0*b(i)*h(i)) / (3.0d0*h(i)**2)
    end do
    return
  end if
  
  ! spline interpolation
  ! mode == 0
  if (x <= xp(1)) then ! outside of the domain - lower
    i1 = 0
  else
    if (x >= xp(np-1)) then ! outside of the domain - upper
      i1 = np - 1
    else
      i1 = 1 ; i2 = np - 1
      do while (i2 - i1 > 1)
        i = (i1 + i2) / 2
        if (x < xp(i)) then
          i2 = i
        else
          i1 = i
        end if
      end do
    end if
  end if
  
  y = (a(i1)*(x - xp(i1))**2 + b(i1)*(x - xp(i1)) + c(i1))*(x - xp(i1)) + yp(i1)
  
end subroutine spline

!------------------------------------------------------------------!
!Tridiagonal matrix algorithm (TDMA)
!a: lower diagonal
!b: main diagonal
!c: upper diagonal
!d: source vector
!x: solution vector
!
!------------------------------------------------------------------!
subroutine tdma(a,b,c,d,n,x)
  real(8),intent(in) :: a(:),b(:),c(:),d(:)
  real(8),intent(out) :: x(:)
  real(8),allocatable :: G(:),H(:)
  real(8) :: temp
  integer,intent(in) :: n
  integer :: i
  
  allocate(G(n),H(n))
  
  !forward elimination
  G(1) = -c(1) / b(1)
  H(1) = d(1) / b(1)
  do i = 2,n
    temp = b(i) + a(i) * G(i-1)
    G(i) = -c(i) / temp
    H(i) = (d(i) - a(i) * H(i-1)) / temp
  end do
  
  !backward substitution
  x(n) = H(n)
  do i = n-1,1,-1
    x(i) = G(i) * x(i+1) + H(i)
  end do
end subroutine tdma

end module Standard_Collection