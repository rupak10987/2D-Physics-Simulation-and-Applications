#include<iostream>
#include<time.h>
#include<graphics.h>
#include<math.h>
struct VEC2
{
    float X_Pos;
    float Y_Pos;
};
struct VEC3
{
    int r=255;
    int g=255;
    int b=255;
};
class verlet_balls
{
public:
    bool is_static=false;
    int m_id;
    float rad;
    struct VEC2 current_pos;
    struct VEC2 prev_pos;
    struct VEC2 vel_dt;
    struct VEC2 acceleration= {0,980}; //(meter/sec^2)
    verlet_balls(class verlet_balls* m_balls[],int unique_indx,float r,struct VEC2 pos)
    {
        this->m_id=unique_indx;
        m_balls[unique_indx]=this;
        this->rad=r;
        this->current_pos=pos;
        this->prev_pos=pos;
    }
    void update_pos(float dt,class verlet_balls* m_balls[],int num_of_balls)
    {
        if(!is_static)
        {
            this->container_pot();
            for(int i=0; i<5; i++) //substeps
                resolve_collision(m_balls,num_of_balls);
            vel_dt= {current_pos.X_Pos-prev_pos.X_Pos,current_pos.Y_Pos-prev_pos.Y_Pos}; //s=vt
            prev_pos=current_pos;
            current_pos= {(current_pos.X_Pos+vel_dt.X_Pos+acceleration.X_Pos*dt*dt),(current_pos.Y_Pos+vel_dt.Y_Pos+acceleration.Y_Pos*dt*dt)};
        }
    }

    void container_pot()
    {
        struct VEC2 pot_pos= {500,500};
        float pot_rad=450;

        struct VEC2 to_obj= {this->current_pos.X_Pos-pot_pos.X_Pos,this->current_pos.Y_Pos-pot_pos.Y_Pos};
        float dist= sqrt(to_obj.X_Pos*to_obj.X_Pos + to_obj.Y_Pos*to_obj.Y_Pos);
        struct VEC2 n_hat= {to_obj.X_Pos/dist,to_obj.Y_Pos/dist};

        if(dist>pot_rad-this->rad)
        {
            this->current_pos= {this->current_pos.X_Pos+(dist-pot_rad+this->rad)*(-n_hat.X_Pos), this->current_pos.Y_Pos+(dist-pot_rad+this->rad)*(-n_hat.Y_Pos)};
        }
    }


    void resolve_collision(class verlet_balls* m_balls[],int num_of_balls)
    {
        for(int i=0; i<num_of_balls; i++)
        {
            if(m_balls[i]->m_id!=this->m_id)
            {
                struct VEC2 to_obj= {this->current_pos.X_Pos-m_balls[i]->current_pos.X_Pos,this->current_pos.Y_Pos-m_balls[i]->current_pos.Y_Pos};
                float dist= sqrt(to_obj.X_Pos*to_obj.X_Pos + to_obj.Y_Pos*to_obj.Y_Pos);
                struct VEC2 n_hat= {to_obj.X_Pos/dist,to_obj.Y_Pos/dist};
                float overlap=(this->rad+m_balls[i]->rad)-dist;
                if(dist<this->rad+m_balls[i]->rad)
                {
                    this->current_pos= {this->current_pos.X_Pos+(0.6*overlap*n_hat.X_Pos), this->current_pos.Y_Pos+(0.6*overlap*n_hat.Y_Pos)};
                    m_balls[i]->current_pos= {m_balls[i]->current_pos.X_Pos-(0.6*overlap*n_hat.X_Pos), m_balls[i]->current_pos.Y_Pos-(0.6*overlap*n_hat.Y_Pos)};
                }
            }
        }
    }


};

class LINKS
{
public:
    class verlet_balls* ball1;
    class verlet_balls* ball2;
    float link_length;
    LINKS(class verlet_balls* m_b1,class verlet_balls* m_b2,float length_of_link,class LINKS* all_links[],int uniq_indx)
    {
        this->ball1=m_b1;
        this->ball2=m_b2;
        this->link_length=length_of_link;
        all_links[uniq_indx]=this;
    }
    virtual void update(float dt)
    {
        struct VEC2 to_obj= {this->ball1->current_pos.X_Pos-this->ball2->current_pos.X_Pos,this->ball1->current_pos.Y_Pos-this->ball2->current_pos.Y_Pos};
        float dist= sqrt(to_obj.X_Pos*to_obj.X_Pos + to_obj.Y_Pos*to_obj.Y_Pos);
        struct VEC2 n_hat= {to_obj.X_Pos/dist,to_obj.Y_Pos/dist};
        float overlap=this->link_length-dist;
        if(ball1->is_static==false)
            this->ball1->current_pos= {this->ball1->current_pos.X_Pos+(dt*dt*3500*overlap*n_hat.X_Pos), this->ball1->current_pos.Y_Pos+(dt*dt*3500*overlap*n_hat.Y_Pos)};
        else
            this->ball2->current_pos= {this->ball2->current_pos.X_Pos-(dt*dt*1750*overlap*n_hat.X_Pos), this->ball2->current_pos.Y_Pos-(dt*dt*1750*overlap*n_hat.Y_Pos)};
        if(ball2->is_static==false)
            this->ball2->current_pos= {this->ball2->current_pos.X_Pos-(dt*dt*3500*overlap*n_hat.X_Pos), this->ball2->current_pos.Y_Pos-(dt*dt*3500*overlap*n_hat.Y_Pos)};
        else
            this->ball1->current_pos= {this->ball1->current_pos.X_Pos+(dt*dt*1750*overlap*n_hat.X_Pos), this->ball1->current_pos.Y_Pos+(dt*dt*1750*overlap*n_hat.Y_Pos)};
    }
};



class SOFT_LINKS:public LINKS
{
public:
    float k=0.01;
    float min_length;
    SOFT_LINKS(class verlet_balls* m_b1,class verlet_balls* m_b2,float max_length_of_link,float min_length_of_the_link,class LINKS* all_links[],int uniq_indx)
        :LINKS(m_b1,m_b2,max_length_of_link,all_links,uniq_indx)
    {
        this->min_length=min_length_of_the_link;
    };
    virtual void update(float dt)
    {
        struct VEC2 to_obj= {this->ball1->current_pos.X_Pos-this->ball2->current_pos.X_Pos,this->ball1->current_pos.Y_Pos-this->ball2->current_pos.Y_Pos};
        float dist= sqrt(to_obj.X_Pos*to_obj.X_Pos + to_obj.Y_Pos*to_obj.Y_Pos);
        struct VEC2 n_hat= {to_obj.X_Pos/dist,to_obj.Y_Pos/dist};
        float overlap=0;
        if(dist>link_length)
            overlap=this->link_length-dist;
        else if(dist<min_length)
            overlap=this->min_length-dist;
        this->ball1->current_pos= {this->ball1->current_pos.X_Pos+(dt*dt*3000*overlap*n_hat.X_Pos), this->ball1->current_pos.Y_Pos+(dt*dt*3000*overlap*n_hat.Y_Pos)};
        this->ball2->current_pos= {this->ball2->current_pos.X_Pos-(dt*dt*3000*overlap*n_hat.X_Pos), this->ball2->current_pos.Y_Pos-(dt*dt*3000*overlap*n_hat.Y_Pos)};

    }
};



int main()
{
    srand(time(0));
    //initialize window
    int num_of_balls=0;
    int gd = DETECT, gm;
    initwindow(1000, 1000);
    POINT cursor_pos;
    //balls for links
    class verlet_balls* v_balls[100];
    for(int  i=0; i<33; i++)
    {
        float random_rad=10;//-rand()%15;
        new verlet_balls(v_balls,i,random_rad, {280+((i%8)*35),450+(i/8)*35});
        num_of_balls++;
    }
    class LINKS* all_links[100];
    int number_of_links=0;


    for(int i=0; i<4; i++)
    {
        if(i<3)
        {
            for(int j=0; j<7; j++)
            {
                new SOFT_LINKS(v_balls[i*8+j],v_balls[((i+1)*8)+j],20,25,all_links,number_of_links);//vertical
                number_of_links++;
                new SOFT_LINKS(v_balls[i*8+j],v_balls[i*8+j+1],20,25,all_links,number_of_links);//horizontal
                number_of_links++;
                new SOFT_LINKS(v_balls[i*8+j],v_balls[(i+1)*8+j+1],20,25,all_links,number_of_links);//diagonal->'\'
                number_of_links++;
                new SOFT_LINKS(v_balls[i*8+j+1],v_balls[(i+1)*8+j],20,25,all_links,number_of_links);//diagonal->'/'
                number_of_links++;
                if(j==6)
                {
                    new SOFT_LINKS(v_balls[i*8+j+1],v_balls[((i+1)*8)+j+1],20,25,all_links,number_of_links);//vertical[right most]
                    number_of_links++;
                }
            }
        }
        else
        {
            for(int j=0; j<7; j++)
            {
                new SOFT_LINKS(v_balls[i*8+j],v_balls[i*8+j+1],20,25,all_links,number_of_links);//horizontal [bottom most]
                number_of_links++;
            }

        }
    }


    //normal balls
    for(int  i=32; i<32; i++)
    {
        float random_rad=20-rand()%15;
        new verlet_balls(v_balls,i,random_rad, {280+((i%4)*120),150+(i/4)*55});
        num_of_balls++;
    }
    v_balls[num_of_balls-1]->is_static=true;
    v_balls[num_of_balls-1]->rad=50;




    float deltaTime;
    float oldtime=clock();//simulation only works good at fixed delta time
    while(true)
    {
        swapbuffers();
        cleardevice();
        deltaTime=clock()-oldtime;
        oldtime=clock();
        deltaTime/=1000.0;
        //if(deltaTime>0.02)
            //deltaTime=0.02;
        circle(500,500,450);
        GetCursorPos(&cursor_pos);
        v_balls[num_of_balls-1]->current_pos= {cursor_pos.x,cursor_pos.y};
        for(int i=0; i<num_of_balls; i++)
        {
            v_balls[i]->update_pos(deltaTime,v_balls,num_of_balls);

            circle(v_balls[i]->current_pos.X_Pos,v_balls[i]->current_pos.Y_Pos,1);
            circle(v_balls[i]->current_pos.X_Pos,v_balls[i]->current_pos.Y_Pos,2);
            putpixel(v_balls[i]->current_pos.X_Pos,v_balls[i]->current_pos.Y_Pos,RED);
            circle(v_balls[i]->current_pos.X_Pos,v_balls[i]->current_pos.Y_Pos,v_balls[i]->rad);
        }
        for(int j=0; j<number_of_links; j++)
        {
            //substeps
            all_links[j]->update(deltaTime);
            line(all_links[j]->ball1->current_pos.X_Pos,all_links[j]->ball1->current_pos.Y_Pos,all_links[j]->ball2->current_pos.X_Pos,all_links[j]->ball2->current_pos.Y_Pos);
        }
    }
    getch();
    closegraph();
    return  0;
}

