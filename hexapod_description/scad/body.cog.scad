module body()
{

  /*[[[cog

  import cog
  import yaml
  
  cog.outl()  
  cog.outl('Pil = {0};'.format(PI_L))
  cog.outl('Piw = {0};'.format(PI_W))
  cog.outl()  
  cog.outl('Bl  = {0};'.format(B_L))
  cog.outl('Bw  = {0};'.format(B_W))
  cog.outl('Bwp = {0};'.format(B_WP))  
  cog.outl()
  cog.outl('h   = {0};'.format(H))
  
  ]]]*/
  //[[[end]]]
    
  v1 = [-(Pil/2+Bl), Piw/2];
  v2 = [-Pil/2, Piw/2+Bw];
  v3 = [Pil/2, Piw/2+Bw];
  v4 = [(Pil/2+Bl), Piw/2];
  v5 = [(Pil/2+Bl), -Piw/2];
  v6 = [Pil/2, -(Piw/2+Bw)];
  v7 = [-Pil/2, -(Piw/2+Bw)];
  v8 = [-(Pil/2+Bl), -Piw/2];
  
  linear_extrude( height = h )
    polygon([v1,v2,v3,v4,v5,v6,v7,v8]);

}

body();
