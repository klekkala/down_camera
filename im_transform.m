function [ret] = im_transform(im, s_x, s_y, t_x, t_y, sh_x, sh_y, theta)

 %Translation matrix
tform = affine2d([1 0 0; 0 1 0; t_x t_y 1]);

 %Rotation matrix
rform = affine2d([cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1]);


 %Scaling matrix
scform = affine2d([s_x 0 0; 0 s_y 0; 0 0 1]);

 %Shearing matrix
shform = affine2d([1 sh_x 0; sh_y 1 0; 0 0 1]);
cb = im;
cb_ref = imref2d(size(cb));
[cb_translated1,cb_translated_ref1] = imwarp(im, tform);
[cb_translated2,cb_translated_ref2] = imwarp(im, rform);
[cb_translated3,cb_translated_ref3] = imwarp(im, scform);
[cb_translated4,cb_translated_ref4] = imwarp(im, shform);

figure;
subplot(3,2,1);
imshow(cb,cb_ref);
subplot(3,2,2);
imshow(cb_translated1,cb_translated_ref1);
subplot(3,2,3);
imshow(cb_translated2,cb_translated_ref2);
subplot(3,2,4);
imshow(cb_translated3,cb_translated_ref3);
subplot(3,2,5);
imshow(cb_translated4,cb_translated_ref4);

final = imwarp(im, tform);
final = imwarp(final, rform);
final = imwarp(final, scform);
[final, final_ref] = imwarp(final, shform);
subplot(3,2,6);
imshow(final,final_ref);
end
