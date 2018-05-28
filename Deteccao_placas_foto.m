
%%---------------------------------
% BLU3040 - Visao Computacional em Robotica
% Avaliacao 2 - Deteccao de placas
% Requisitos:
%   MATLAB
%   Machine Vision Toolbox 
%       P.I. Corke, â€œRobotics, Vision & Controlâ€?, Springer 2011, ISBN 978-3-642-20143-1.
%       http://petercorke.com/wordpress/toolboxes/machine-vision-toolbox
%%---------------------------------

clear all
close all
clc

placa1 = iread('dataset/placa_carro_h1.jpg', 'gray');
%placa1 = iread('dataset/placa_carro4.jpg', 'gray');
letras = iread('dataset/Letras.jpg', 'gray');
numeros = not(iread('dataset/Numeros.png', 'gray'));

alfabeto = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z'];
numero = ['0','1','2','3','4','5','6','7','8','9'];

t = otsu(placa1);
mask = placa1<t-30;

[a b] = size(placa1);

if((b/a)>2.9 && (b/a)<3.09)
    vcort = floor(size(placa1,1)*0.13846)+5;
    vcort2 = floor(size(placa1,1)*0.292307)+5;
    vcort3 = floor(size(placa1,1)*0.9);
    ucort = floor(size(placa1,2)*0.08)+20;
    ucort2 = floor(size(placa1,2)*0.84)-5;
    
    placa2 = placa1;
    
else
    blobs = iblobs(mask, 'area', [a*b*3.0832e-04, a*b*0.6166]);
    
    for i=1:size(blobs,2)
        if(size(blobs(i).children,2) >= 5)
            n_placa = i;
        end
    end
    
    l = ilabel(mask);
    
    for i=1:blobs(n_placa).umax
        if(l(blobs(n_placa).vmin,i) == blobs(n_placa).label)
            x1 = i;
            y1 = blobs(n_placa).vmin;
        end
        if(l(blobs(n_placa).vmax,i) == blobs(n_placa).label)
            x2 = i;
            y2 = blobs(n_placa).vmax;
        end
    end
    
    for i=1:blobs(n_placa).vmax
        if(l(i,blobs(n_placa).umin) == blobs(n_placa).label)
            y3 = i;
            x3 = blobs(n_placa).umin;
        end
        if(l(i,blobs(n_placa).umax) == blobs(n_placa).label)
            y4 = i;
            x4 = blobs(n_placa).umax;
        end
    end
    
    p1 = [x1 x2 x3 x4; y1 y2 y3 y4];
    
    umin = blobs(n_placa).umin;
    umax = blobs(n_placa).umax;
    du = umax-umin;
    
    dv=du*0.325;
    vmax = blobs(n_placa).vmax;
    vmin = vmax - dv;
    
    if(blobs(n_placa).theta < 0)
        p2 = [umax umin umin umax; vmin vmax vmin vmax];
    elseif(blobs(n_placa).theta > 0)
        p2 = [umin umax umin umax; vmin vmax vmax vmin];
    elseif(blobs(n_placa).theta == 0)
        p1 = [0 size(placa1,2) size(placa1,2) 0; 0 0 size(placa1,1) size(placa1,1)];
        p2 = p1;
    end
    
    H = homography(p1, p2);
    
    placa1_h = homwarp(H, placa1);
    
    placa1_h = iint(placa1_h);
    
    klap = klog(2);
    out1 = iconvolve(placa1_h, klap);
    
    ht = Hough(out1, 'houghthresh', 0.9);
    l = ht.lines();
    
    soma = 0;
    for i=1:size(l,2)
        soma = soma + l(i).theta;
    end
    
    theta = soma/size(l,2);
    thetad = theta*180/pi;
    
    placa1_h = irotate(placa1_h, thetad);
    
    th = otsu(placa1_h);
    maskh = placa1_h > th;
    blobs22 = iblobs(maskh, 'class', 1, 'area', [15, 10000000]);
    arearef = 0;
    for i = 1:size(blobs22,2)
        if(blobs22(i).area > arearef)
            arearef = blobs22(i).area;
            n_placa22 = i;
        end
    end
    
    vmin = blobs22(n_placa22).vmin;
    
    placa2 = placa1_h([floor(vmin) : floor(vmax)+10], [floor(umin) : floor(umax)]);
    
    vcort = floor(size(placa2,1)*0.16946);
    vcort2 = floor(size(placa2,1)*0.348307);
    vcort3 = floor(size(placa2,1)*0.9);
    ucort = floor(size(placa2,2)*0.2);
    ucort2 = floor(size(placa2,2)*0.76);
end
    

p_cidade = placa2([vcort:vcort2],[ucort:ucort2]);
p_letras = placa2([vcort2:vcort3],:);

t = 80;
t1 = otsu(p_cidade)-4;
t2 = otsu(p_letras);

mask1 = p_cidade<t1;
%mask1 = iopen(mask1, [1 1 1; 1 1 1; 1 1 1]);
%mask1 = iclose(mask1, [1 1 1; 1 1 1; 1 1 1]);
mask2 = p_letras<t2;
letras = letras<t;

amin1 = size(mask1,1)*size(mask1,2)*0.00288;
amax1 = size(mask1,1)*size(mask1,2)*0.01970;

amin2 = size(mask2,1)*size(mask2,2)*0.015238;
amax2 = size(mask2,1)*size(mask2,2)*0.090000;

blobs1 = iblobs(mask1, 'area', [amin1, amax1], 'connect', 8, 'class', 1);
blobs2 = iblobs(mask2, 'area', [amin2, amax2], 'class', 1);

vetor1 = blobs1.box();
u1 = vetor1(1,:);
v1 = vetor1(2,:);

vetor2 = blobs1.class;

vetor4 = blobs2.box();
u2 = vetor4(1,:);
v2 = vetor4(2,:);
 
vetor5 = blobs2.class;
 
s1 = size(blobs1,2);
s2 = size(blobs2,2);

for i=1:s1
     if(vetor2(i) == 1 && (blobs1(i).umax - blobs1(i).umin) < (blobs1(i).vmax - blobs1(i).vmin))
        umin1(i) = u1(i);
        vmin1(i) = v1(i);
    else
        umin1(i) = NaN;
        vmin1(i) = NaN;
     end
 end
 
 umin1 = sort(umin1);
 
for i=1:s1
    for j=1:s1
        if(umin1(i) == u1(j))
            vetlab1(i) = j;
        end
    end
end
 
for i=1:s2
     if(vetor5(i) == 1)
        umin(i) = u2(i);
        vmin(i) = v2(i);
    else
        umin(i) = NaN;
        vmin(i) = NaN;
     end
 end
 
 umin = sort(umin);
 
for i=1:s2
    for j=1:s2
        if(umin(i) == u2(j))
            vetlab(i) = j;
        end
    end
end
 
for i=1:size(vetlab1,2)
    if(vetor2(vetlab1(i)) == 1 && (blobs1(vetlab1(i)).umax - blobs1(vetlab1(i)).umin) < (blobs1(vetlab1(i)).vmax - blobs1(vetlab1(i)).vmin))
        blob_cidade{i} = p_cidade([blobs1(vetlab1(i)).vmin: blobs1(vetlab1(i)).vmax], [blobs1(vetlab1(i)).umin: blobs1(vetlab1(i)).umax]);
        blob_cidade{i} = inormhist(blob_cidade{i});
        t = otsu(blob_cidade{i})-40;
        blob_cidade{i} = blob_cidade{i}<t;
    end
end

 for i=1:size(vetlab,2)
    if(vetor5(vetlab(i)) == 1)
        blob_placa{i} = mask2([v2(vetlab(i)): v2(vetlab(i)+s2)], [u2(vetlab(i)):u2(vetlab(i)+s2)]);
    end
 end
 
 blobsletra = iblobs(letras, 'area', [50, 1000]);
 
 vetor6 = blobsletra.box();
 u3 = vetor6(1,:);
 v3 = vetor6(2,:);
 vetor7 = blobsletra.class;
 
 s3 = size(blobsletra,2);
 
 for i=1:s3
     if(vetor7(i) == 1)
        umin2(i) = u3(i);
        vmin2(i) = v3(i);
    else
        umin2(i) = NaN;
        vmin2(i) = NaN;
     end
 end
 
 umin2 = sort(umin2);
 
for i=1:s3
    for j=1:s3
        if(umin2(i) == u3(j))
            vetlab2(i) = j;
        end
    end
end
 
for i=1:size(vetlab2,2)
     if(vetor7(vetlab2(i)) == 1)
        blob_letra{i} = letras([v3(vetlab2(i)): v3(vetlab2(i)+s3)], [u3(vetlab2(i)): u3(vetlab2(i)+s3)]);
     end
end

blobsnumero = iblobs(numeros, 'area', [100, 1000]);
 
 vetor8 = blobsnumero.box();
 u4 = vetor8(1,:);
 v4 = vetor8(2,:);
 vetor9 = blobsnumero.class;
 
 s4 = size(blobsnumero,2);
 
for i=1:s4
     if(vetor9(i) == 1)
        blob_numero{i} = numeros([v4(i): v4(i+s4)], [u4(i): u4(i+s4)]);
     end
end
 
 
count = zeros(size(blob_letra,2), 3);
count2 = zeros(size(blob_numero,2), 4);
count3 = zeros(size(blob_letra,2), size(blob_cidade,2));
count4 = zeros(size(blob_cidade,2),size(blob_cidade,2)-2);
t1 = 150;
t2 = 180;
zero = zeros(100,70);

for j=1:3
    for i=1:size(blob_letra,2)
        out1 = isamesize(blob_letra{1,i}, zero);
        out2 = isamesize(blob_placa{1,j}, zero);
        out1 = out1>t2;
        out2 = out2>t1;
        count(i,j) = zncc(out2,out1);
    end
end

max = [count(1,1), count(1,2), count(1,3)];
posi = [1, 1, 1];

for j=1:3
    for i=2:size(blob_letra,2)
        if(count(i,j) > max(1,j))
            max(1,j) = count(i,j);
            posi(1,j) = i;
        end
    end
end

for j=4:7
    for i=1:size(blob_numero,2)
        out3 = isamesize(blob_numero{1,i}, zero);
        out4 = isamesize(blob_placa{1,j}, zero);
        out3 = out3>t2;
        out4 = out4>t1;
        count2(i,j-3) = zncc(out4,out3);
    end
end

max2 = [count2(1,1), count2(1,2), count2(1,3), count2(1,4)];
posin = [1, 1, 1, 1];

for j=1:4
    for i=2:size(blob_numero,2)
        if(count2(i,j) > max2(1,j))
            max2(1,j) = count2(i,j);
            posin(1,j) = i;
        end
    end
end

for j=1:size(blob_cidade,2)
    for i=1:size(blob_letra,2)
        out5 = isamesize(blob_cidade{1,j}, zero);
        out6 = isamesize(blob_letra{1,i}, zero);
        t5 = otsu(out5);
        t6 = 110;
        out7 = out5>t5;
        out8 = out6>t6;
        count3(i,j) = zncc(out7, out8);
    end
end

max3 = [count3(1,:)];
posic =ones([1,size(blob_cidade,2)]);

for j=1:size(blob_cidade,2)
    for i=2:size(blob_letra,2)
        if(count3(i,j) > max3(1,j))
            max3(1,j) = count3(i,j);
            posic(1,j) = i;
        end
    end
end

letras_placa = strcat(alfabeto(posi(1)), alfabeto(posi(2)), alfabeto(posi(3)));
numeros_placa = strcat(numero(posin(1)), numero(posin(2)), numero(posin(3)), numero(posin(4)));
cidade_placa = ' ';
count5 = 0;

for i=1:size(blob_cidade,2)
    cidade_placa = strcat(cidade_placa, alfabeto(posic(i)));
    count5 = count5+1;
    if(count5 == 2)
        cidade_placa = strcat(cidade_placa, '_');
    end
end
