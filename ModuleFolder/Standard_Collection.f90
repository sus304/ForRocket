module Standard_Collection
implicit none

real(8),parameter :: PI = 3.141592d0 , R = 287.1d0 , gamma = 1.4d0

real(8),parameter :: Re = 6356766.0d0 !---地球半径--
real(8),parameter :: GT = 0.0065d0 !-------気温減衰率K/m--------

contains

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