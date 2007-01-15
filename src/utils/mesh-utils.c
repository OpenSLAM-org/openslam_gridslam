#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <values.h>

#include <navigation/utils.h>

#define AVG_NUM_LINKS                10

void
mesh_from_spf( SPF_POINTS_DATA scan, MESH_DATA * mesh )
{
  int                i, n, p, r, used;
  int                pn, n1, n2;
  int              * nlinks;
  int              * ulinks;
  int             ** lset;
  
  /* count the degree of the nodes */
  nlinks = (int *) malloc( scan.numpoints * sizeof(int) );
  for (n=0; n<scan.numpoints; n++) {
    nlinks[n] = 0;
  }

  /* table of existing links */
  lset  = (int **) malloc( sizeof(int *) * scan.numpoints );
  ulinks = (int *) malloc( sizeof(int) * scan.numpoints );
  
  for (n=0; n<scan.numpoints; n++) {
    lset[n] = (int *) malloc( sizeof(int) * AVG_NUM_LINKS );
    ulinks[n] = AVG_NUM_LINKS;
  }
  
  for (n=0; n<scan.numpolygons; n++) {
    for (p=0; p<scan.polygon[n].numfaces; p++) {
      pn = (p+1)%scan.polygon[n].numfaces;
      n1 = scan.polygon[n].facept[p];
      n2 = scan.polygon[n].facept[pn];
      /* safety check ... */
      if ( n1>=0 && n1<scan.numpoints &&
	   n2>=0 && n2<scan.numpoints ) {
	/* forward direction */
	used = FALSE;
	for (i=0; i<nlinks[n1]; i++) {
	  if (lset[n1][i]==n2) {
	    used = TRUE;
	  }
	}
	if (!used) {
	  if (ulinks[n1]<=AVG_NUM_LINKS) {
	    ulinks[n1] += AVG_NUM_LINKS;
	    lset[n1] =
	      (int *) realloc( lset[n1], sizeof(int) *  ulinks[n1] );
	  }
	  lset[n1][nlinks[n1]] = n2;
	  nlinks[n1]++;
	}
	/* backward direction */
	used = FALSE;
	for (i=0; i<nlinks[n2]; i++) {
	  if (lset[n2][i]==n1) {
	    used = TRUE;
	  }
	}
	if (!used) {
	  if (ulinks[n2]<=AVG_NUM_LINKS) {
	    ulinks[n2] += AVG_NUM_LINKS;
	    lset[n2] =
	      (int *) realloc( lset[n2], sizeof(int) *  ulinks[n2] );
	  }
	  lset[n2][nlinks[n2]] = n1;
	  nlinks[n2]++;
	}
      }
    }    
  }

  /* alloc the nodes */
  mesh->nodes = (NODE *) malloc( scan.numpoints * sizeof(NODE) );
  mesh->numnodes = scan.numpoints;
  
  for (n=0; n<scan.numpoints; n++) {
    
    mesh->nodes[n].id             = n;

    mesh->nodes[n].x              = scan.point[n].x;
    mesh->nodes[n].y              = scan.point[n].y;
    mesh->nodes[n].z              = scan.point[n].z;

    mesh->nodes[n].roll           = 0;
    mesh->nodes[n].pitch          = 0;
    mesh->nodes[n].yaw            = 0;

    mesh->nodes[n].lnk            = (LINK *) malloc(nlinks[n] * sizeof(LINK));
    mesh->nodes[n].numlinks       = 0;

  }
  
  for (n=0; n<scan.numpolygons; n++) {
    for (p=0; p<scan.polygon[n].numfaces; p++) {
      
      pn = (p+1)%scan.polygon[n].numfaces;
      n1 = scan.polygon[n].facept[p];
      n2 = scan.polygon[n].facept[pn];
      
      if ( n1>=0 && n1<scan.numpoints &&
	   n2>=0 && n2<scan.numpoints ) {
	
	/* forward direction */
	used = FALSE;
	r = mesh->nodes[n1].numlinks;
	for (i=0; i<r; i++) {
	  if (lset[n1][i]==n2) {
	    used = TRUE;
	  }
	}
	if (!used) {
	  mesh->nodes[n1].lnk[r].from = n1;
	  mesh->nodes[n1].lnk[r].to   = n2;
	  /*
	    mesh->nodes[n1].lnk[mesh->nodes[n1].numlinks];
  	    .calculate((*nodes)[n1], (*nodes)[n2]);
	  */
	  lset[n1][r] = n2;
	  mesh->nodes[n1].numlinks++;
	}
	/* backward direction */
	used = FALSE;
	r = mesh->nodes[n2].numlinks;
	for (i=0; i<r; i++) {
	  if (lset[n2][i]==n1) {
	    used = TRUE;
	  }
	}
	if (!used) {
	  mesh->nodes[n2].lnk[r].from = n2;
	  mesh->nodes[n2].lnk[r].to   = n1;
	  /*
	    mesh->nodes[n2].lnk[mesh->nodes[n2].numlinks];
	    .calculate((*nodes)[n2], (*nodes)[n1]);
	  */
	  lset[n2][r] = n1;
	  mesh->nodes[n2].numlinks++;
	}
      }
    }
  }   
  
  for (n=0; n<scan.numpoints; n++) {
    free(lset[n]);
  }
  free(lset);
  free(ulinks);
  free(nlinks);

}

int
mesh_components( MESH_DATA mesh, POINTREF_SET ** components )
{
  int                      i, id, sn;
  int                    * seen;

  void DFS( int index ) {
    int l;
    if (!seen[index]) {
      seen[index] = id;
      for (l=0; l<mesh.nodes[index].numlinks; l++) {
	DFS( mesh.nodes[index].lnk[l].to );
      }
    }
  }
  
  if (mesh.numnodes == 0)
    return(0);

  seen =
    (int *) malloc( mesh.numnodes * sizeof(int) );

  for (i=0; i<mesh.numnodes; i++) {
    seen[i] = FALSE;
  }

  id = 1;
  for (i=0; i<mesh.numnodes; i++) {
    if (!seen[i]) {
      DFS( i );
      id++;
    }
  }
  id--;
  
  *components = (POINTREF_SET *) malloc( id * sizeof(POINTREF_SET) );

  for (i=0; i<id; i++) {
    (*components)[i].numpoints = 0;
  }
  for (i=0; i<mesh.numnodes; i++) {
    (*components)[seen[i]-1].numpoints++;
  }
  for (i=0; i<id; i++) {
    (*components)[i].ptref =
      (int *) malloc( (*components)[i].numpoints * sizeof(int) );
    (*components)[i].numpoints = 0;
  }
  for (i=0; i<mesh.numnodes; i++) {
    sn = seen[i]-1;
    (*components)[sn].ptref[(*components)[sn].numpoints] = i;
    (*components)[sn].numpoints++;
  }

  free(seen);

  return( id );
}
		      
void
spf_get_component( SPF_POINTS_DATA spf, POINTREF_SET component,
		   SPF_POINTS_DATA * spf_comp )
{
  int       c, i, j, r, pc, use;
  int     * valid, * ref, *valid2;

  valid = (int *) malloc( spf.numpoints * sizeof(int) );
  ref = (int *) malloc( spf.numpoints * sizeof(int) );
  
  for (i=0; i<spf.numpoints; i++) {
    valid[i] = FALSE;
  }
  
  c = 0;
  for (i=0; i<component.numpoints; i++) {
    r = component.ptref[i];
    if (r>=0 && r<spf.numpoints) {
      valid[r] = TRUE;
      c++;
    }
  }

  spf_comp->numpoints = c;
  spf_comp->point = (POINT3 *) malloc( c * sizeof(POINT3) );

  c = 0;
  for (i=0; i<spf.numpoints; i++) {
    ref[i] = -1;
    if (valid[i]) {
      spf_comp->point[c] = spf.point[i];
      ref[i] = c;
      c++;
    }
  }

  valid2 = (int *) malloc( spf.numpolygons * sizeof(int) );
  for (i=0; i<spf.numpolygons; i++) {
    valid2[i] = FALSE;
  }

  pc = 0;
  for (i=0; i<spf.numpolygons; i++) {
    use = TRUE;
    for (j=0; j<spf.polygon[i].numfaces; j++) {
      r = spf.polygon[i].facept[j];
      if (r<0 || r>=spf.numpoints || !valid[r]) {
	use = FALSE;
      }
    }
    if (use) {
      valid2[i] = TRUE; 
      pc++;
    }
  }

  spf_comp->numpolygons = pc;
  spf_comp->polygon = (POLYGON_REF *) malloc( pc * sizeof(POLYGON_REF) );

  pc = 0;
  for (i=0; i<spf.numpolygons; i++) {
    if (valid2[i]) {
      spf_comp->polygon[pc].numfaces = spf.polygon[i].numfaces;
      spf_comp->polygon[pc].facept =
	(int *) malloc ( spf.polygon[i].numfaces * sizeof(int) );
      for (j=0; j<spf.polygon[i].numfaces; j++) {
	spf_comp->polygon[pc].facept[j] = ref[spf.polygon[i].facept[j]];
      }
      pc++;
    }
  }

  spf_comp->numplanes = 0;
  spf_comp->plane = NULL;

  free(valid);
  free(valid2);
  free(ref);
  
}

void
spf_filter_small_components( SPF_POINTS_DATA spf, int num_components, POINTREF_SET *components,
			     int min_comp_size, SPF_POINTS_DATA * spf_comp )
{
  int       c, i, j, r, pc, use;
  int     * valid, * ref, *valid2;

  valid = (int *) malloc( spf.numpoints * sizeof(int) );
  ref = (int *) malloc( spf.numpoints * sizeof(int) );
  
  for (i=0; i<spf.numpoints; i++) {
    valid[i] = FALSE;
  }
  
  c = 0;
  for (j=0; j<num_components; j++) {
    if (components[j].numpoints>=min_comp_size) {
      for (i=0; i<components[j].numpoints; i++) {
	r = components[j].ptref[i];
	if (r>=0 && r<spf.numpoints) {
	  valid[r] = TRUE;
	  c++;
	}
      }
    } else {
      fprintf( stderr, "filter comp %d with %d points\n", j, components[j].numpoints );
    }
  }

  spf_comp->numpoints = c;
  spf_comp->point = (POINT3 *) malloc( c * sizeof(POINT3) );

  c = 0;
  for (i=0; i<spf.numpoints; i++) {
    ref[i] = -1;
    if (valid[i]) {
      spf_comp->point[c] = spf.point[i];
      ref[i] = c;
      c++;
    }
  }

  valid2 = (int *) malloc( spf.numpolygons * sizeof(int) );
  for (i=0; i<spf.numpolygons; i++) {
    valid2[i] = FALSE;
  }

  pc = 0;
  for (i=0; i<spf.numpolygons; i++) {
    use = TRUE;
    for (j=0; j<spf.polygon[i].numfaces; j++) {
      r = spf.polygon[i].facept[j];
      if (r<0 || r>=spf.numpoints || !valid[r]) {
	use = FALSE;
      }
    }
    if (use) {
      valid2[i] = TRUE; 
      pc++;
    }
  }

  spf_comp->numpolygons = pc;
  spf_comp->polygon = (POLYGON_REF *) malloc( pc * sizeof(POLYGON_REF) );

  pc = 0;
  for (i=0; i<spf.numpolygons; i++) {
    if (valid2[i]) {
      spf_comp->polygon[pc].numfaces = spf.polygon[i].numfaces;
      spf_comp->polygon[pc].facept =
	(int *) malloc ( spf.polygon[i].numfaces * sizeof(int) );
      for (j=0; j<spf.polygon[i].numfaces; j++) {
	spf_comp->polygon[pc].facept[j] = ref[spf.polygon[i].facept[j]];
      }
      pc++;
    }
  }

  spf_comp->numplanes = 0;
  spf_comp->plane = NULL;

  free(valid);
  free(valid2);
  free(ref);
  
}
