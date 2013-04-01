celinux
=======

Consumer Electronics Linux (CELinux)

Note about choice of compiler:
At the time that Linux 2.4.26 was released (2004-04-14), the latest
released version of gcc was 3.3.3, with 3.4.0 just around the corner
(2004-04-18.)  gcc 4.1.1 wouldn't even exist for another two years
(2006-05-24) so it's not surprising that this combination isn't
compatible because gcc tends to get stricter over time.  Those errors
seem to refer to a gcc extension that was deprecated starting in 3.4 and
apparently removed by the time of 4.1.

You can most likely work around this without too much fuss, but IMO
you'd be better off using versions that were historically coincident. 
That way you get free validation that it's a stable combination, whereas
with time-machine mix-and-match you have to determine that yourself. 

