%{
a, b : due versori in R^3

OSSERVAZIONI:
|a X b| = sin(th)
a . b = cos(th)

quindi 
th = arctan2( sin, cos )
rho = versore(a X b) * th

nello script rho = c


si scrive (sinth > 0.00000000001)
si legge "a, b non paralleli"
%}

function c = ReducedVersorLemma( a,  b )

vsinth = cross(a,b);
costh = dot(a,b);

sinth = norm(vsinth,2);

if  (sinth > 0.00000000001)
    theta = atan2(sinth,costh);
    c = (vsinth * (theta/sinth));
else
    if (costh>0)
        c = 0;
    else 
        c = 0;
    end
end

end

