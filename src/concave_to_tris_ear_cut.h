namespace concave_to_tris
{
struct VEC2
{
    float X_Pos;
    float Y_Pos;
};

bool is_clock_wise(struct VEC2 A,struct VEC2 B,struct VEC2 C);
bool is_point_inside_triangle(struct VEC2 A,struct VEC2 B,struct VEC2 C,struct VEC2 P);
void adjust_array(float *a,int* array_length,int drop_index);
void form_tris(float *tris);
void mid_of_triangle(float *tris_vert);
void split_concave_poly_into_tris(float *a,int *ar_length,float *tris[]);

void split_concave_poly_into_tris(float *a,int *ar_length,float *tris[])
{
    int og_length=*ar_length;
    //copy the vertex data in another array for inside point check
    float copy_a[*ar_length];
    for(int i=0; i<*ar_length; i++)
    {
        copy_a[i]=a[i];
    }

    int tri_counter=0;

    while(true)
    {
    for(int i=0; i<*ar_length; i+=2)
    {
        //indexing A,B,C
        int ind_a,ind_b,ind_c;
        ind_a=(i-2);
        if(ind_a<0)
            ind_a=*ar_length-2;
        ind_b=i;
        ind_c=(i+2)%*ar_length;
        //check orientaion of ABC, ear?
        bool clockW=is_clock_wise({a[ind_a],a[ind_a+1]}, {a[ind_b],a[ind_b+1]}, {a[ind_c],a[ind_c+1]});
        //std::cout<<"vert"<<ind_b<<" clock = "<<clockW<<"\n";
        //if ear then check if any points of the og convex inside it
        if(clockW)
        {
            bool inside=false;
            for(int p=0;p<og_length;p+=2)
            {
                inside|=is_point_inside_triangle({a[ind_a],a[ind_a+1]}, {a[ind_b],a[ind_b+1]}, {a[ind_c],a[ind_c+1]},{copy_a[p],copy_a[p+1]});
            }
            if(!inside)//its an ear
            {
                //form the triangle
                tris[tri_counter][0]=a[ind_a];
                tris[tri_counter][1]=a[ind_a+1];
                tris[tri_counter][2]=a[ind_b];
                tris[tri_counter][3]=a[ind_b+1];
                tris[tri_counter][4]=a[ind_c];
                tris[tri_counter][5]=a[ind_c+1];
                mid_of_triangle(tris[tri_counter]);
                tri_counter++;

               // std::cout<<"ind"<<ind_b<<" is clipped\n";
                //clip it
                adjust_array(a,ar_length,ind_b);
                break;
            }
        }

    }
    if(*ar_length<=2*2)
    break;
    }
}

void mid_of_triangle(float *tris_vert)
{
    VEC2 A={tris_vert[0],tris_vert[1]};
    VEC2 B={tris_vert[2],tris_vert[3]};
    VEC2 C={tris_vert[4],tris_vert[5]};
    VEC2 BC_BY_2={(B.X_Pos+C.X_Pos)/2.0,(B.Y_Pos+C.Y_Pos)/2.0};
    VEC2 A_to_BC_BY_2={(BC_BY_2.X_Pos+A.X_Pos)/2.0,(BC_BY_2.Y_Pos+A.Y_Pos)/2.0};
    tris_vert[6]=A_to_BC_BY_2.X_Pos*(3/2);
    tris_vert[7]=A_to_BC_BY_2.Y_Pos*(3/2);
};

void adjust_array(float *a,int* array_length,int drop_index)
{
    *array_length-=2;
    for(int i=drop_index; i<*array_length; i++)
    {
        a[i]=a[i+2];
        a[i+1]=a[i+3];
    }
}




bool is_point_inside_triangle(struct VEC2 A,struct VEC2 B,struct VEC2 C,struct VEC2 P)
{
    int O=is_clock_wise(A,B,P)+is_clock_wise(B,C,P)+is_clock_wise(C,A,P);
    if(O%3==0)
        return true;
    else
        return false;
}


bool is_clock_wise(struct VEC2 A,struct VEC2 B,struct VEC2 C)
{
    struct VEC2 AB= {B.X_Pos-A.X_Pos,B.Y_Pos-A.Y_Pos};
    struct VEC2 BC= {C.X_Pos-B.X_Pos, C.Y_Pos-B.Y_Pos};

    float ABcrossBC=AB.X_Pos * BC.Y_Pos - AB.Y_Pos * BC.X_Pos;

    if(ABcrossBC<0)
        return true;
    else
        return false;
}
};

