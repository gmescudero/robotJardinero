i=imread('jardinPlanta.png');

imgR=(i(:,:,1) == 0);
imgG=(i(:,:,2) == 77);
imgB=(i(:,:,3) == 0);

fontR=(i(:,:,1) == 255);
fontG=(i(:,:,2) == 255);
fontB=(i(:,:,3) == 255);

housR=(i(:,:,1) == 95);
housG=(i(:,:,2) == 95);
housB=(i(:,:,3) == 95);

img = (imgR & imgG & imgB) | (fontR & fontG & fontB) | (housR & housG & housB);
% imshow(~img);
BW = imfill(img,'holes');

imshow(~BW);