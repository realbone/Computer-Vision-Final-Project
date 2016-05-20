function out=vc_fun(x,y)
out=vech(x*y'+y*x'-diag(x.*y));