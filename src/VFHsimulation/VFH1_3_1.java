package VFHsimulation;
/***********************************************************************
 * Version:1.3.1
 * Author :yin
 * Date   :2016-8-14
 * About  :VFH的仿真程序，周身720个传感器，只使用前面的361个。处理雷达传感器为扇形
 * 区块。找一个尽可能靠近目标的一个方向为决定转向。ladar传感器量程修改为8米
 * ################################ FIX ################################
 * Fix Version:1.0
 * Fix Author :yin
 * Fix Date   :2016-8-13
 * Fix About  :完成基本的程序功能，修正了一些常用的错误，测试运行通过。计算可行区域
 * 的宽度。测试中有碰撞问题。
 * ################################ FIX ################################
 * Fix Version:1.1
 * Fix Author :yin
 * Fix Date   :2016-8-14
 * Fix About  :不计算可行区域的宽度，而是以一个安全半径来减小可行的方向角度，并且
 * 如果可行区域包含到目标的航向，就直接向着目标。如果不是，就走最靠近目标航向的可行
 * 区域边界。实验证明虽然任然会有碰撞问题，但是规划的路径比上一个版本的中线行走更优v，
 * 【这种优化方法比计算宽度的方法更优】
 * ################################ FIX ################################
 * Fix Version:1.2
 * Fix Author :yin
 * Fix Date   :2016-8-18
 * Fix About  :对上一个版本的优化做调试，修改输出数据，把重要的调试参数显示出来。
 * 开启了动态阈值的功能。此版本基本是吴师兄的版本，仅仅是做了减小可行区域角度的优化。
 * 个人认为次版本的创新点在于：1，均值动态阈值（解决了全封闭问题）2，放弃走区块中线
 * 的方法，尽可能靠近目标方位（解决了全开放问题）。在上一版本中本人做了一个减小可行
 * 区域角度的优化。将此版本作为无人车项目验收，仿真视频录制的版本(量程5米，机器人半
 * 径0.3米)。
 * ################################ FIX ################################
 * Fix Version:1.2.1
 * Fix Author :yin
 * Fix Date   :2016-8-18 night
 * Fix About  :修复了0帧机器人速度方向的错误，默认机器人0帧面z正向
 * ################################ FIX ################################
 * Fix Version:1.3
 * Fix Author :yin
 * Fix Date   :2016-8-19
 * Fix About  :
 * 1，如果目标在量程范围内可见，设目标为刚好可见【解决了目标附近有
 * 障碍的情况】
 * 2，预留了全封闭逻辑
 * ################################ FIX ################################
 * Fix Version:1.3.1
 * Fix Author :yin
 * Fix Date   :2016-8-19
 * Fix About  :
 * 1，修改了目标锁定，如果目标可见就锁定
 * 2，证明了锁定目标逻辑的重要性
 * 3，新提出了一种动态阈值的计算方法，目前来看两者的区别不大
 * 4，可行区域的处理带来了二次全封闭（封闭与碰撞之间的矛盾）
 * 5，加入了路径保存功能
 ***********************************************************************/

import simbad.gui.Simbad;
import simbad.sim.*;

import javax.vecmath.*;

import java.io.*;

/**
  Derivate your own code from this example.
 */


public class VFH1_3_1{

    /** Describe the robot */
    static public class Robot extends Agent {

        RangeSensorBelt sonars;


        Vector2d goalcoord = new Vector2d(8,8); //存储目标点
        Vector3d goal3d = new Vector3d(8, 0,8);
        double carwidth=0.3;

        //--------------------计算航向偏角用--------------------
        Point3d nowcoord = new Point3d();//此刻的坐标
        double errorangle;//计算航向偏角

        //--------------------处理ladar数据用--------------------
        double ladar[] = new double[361];//存储机器人ladr数据
        int ladar_bin[] = new int[361];//存储机器人ladr的二值数据
        double ladarmin,ladarmax,ladarmid;

        float liangcheng=5f;//量程
        double m_threshold;//环境动态阈值，其与环境疏密有关

        int numcount = 0;//记录区块数量
        int startcount[] = new int[400];//记录区块起点编号
        int endcount[] = new int[400];//记录区块终点编号
        double startangle[] = new double[400];//记录区块起线弧度
        double endangle[] = new double[400];//记录区块终线弧度
        double midangle[] = new double[400];//记录区块中线弧度

        double minangle;
        double bestturn;//决定转向的弧度
        int bestnum;

        public Robot(Vector3d position, String name) {
            super(position, name);

            // Add sonars
            sonars = RobotFactory.addSonarBeltSensor(this,720);
        }

        /** This method is called by the simulator engine on reset. */
        public void initBehavior() {
            // nothing particular in this case
        }

        /** This method is call cyclically (20 times per second)  by the simulator engine. */
        public void performBehavior() {
        	if(distanceGoal()<0.5)
        	{
        		setRotationalVelocity(0);
				setTranslationalVelocity(0);
        	}
        	else
        	{
        		//saveTrajectory(20);
            	if(getCounter()%20==0)//为防止剧烈晃动，每10帧做一次
            	{System.out.println("-----------------------------------------------------------------");
                    //++++++++思考
        			        	//--------------------处理ladar数据:1,读取传感器数据到数组--------------------
        			        	for(int n=0;n<361;n++)
        			        	{
        			        		ladar[n]=sonars.getMeasurement(getControlcode(n));//获取数据
        			        		if(ladar[n]==Double.POSITIVE_INFINITY)//如果超过量程，处理为量程
        			        			ladar[n]=liangcheng;
        			        	}

        			        	//--------------------处理ladar数据:2,动态阈值处理成二值数组--------------------

        			        	/*ladarmin=ladar[0];//我的均值计算方法
        			        	ladarmax=ladar[0];
        			        	for(int i=0;i<361;i++)
        			        	{
        			        		if(ladar[i]<ladarmin)
        			        			ladarmin=ladar[i];
        			        		if(ladar[i]>ladarmax)
        			        			ladarmax=ladar[i];
        			        	}
        			        	ladarmid=(ladarmin+ladarmax)/2;
        			        	m_threshold = ladarmid;*/

        			        	/*double sum=0;//动态阈值，待调试
        			        	for(int i=0;i<61;i++)//提取6的倍数的点,求和取均值
        			        		sum=sum+ladar[i*6];
        			        	m_threshold=sum/61;*/

        			        	double sum=0;
        			        	  for(int i=0;i<361;i++)//所有点均值
        			        		sum=sum+ladar[i];
        			        	m_threshold=sum/361;

        			        	if(m_threshold>=liangcheng)
        			        		m_threshold = liangcheng;

        			        	if(distanceGoal()<=m_threshold)//如果目标在可见范围内，设目标为刚好可见【锁定目标】
        			        		m_threshold=distanceGoal();


    			        		for(int i=0;i<361;i++)
    			        		{
    			        			if(ladar[i]>=m_threshold)	ladar_bin[i]=0;
    			        			else						ladar_bin[i]=1;
    			        		}
    			        		//均值阈值的情况下，不可能出现全封闭；而非中线走法又隐含了处理全开放的逻辑

        			        	//--------------------处理ladar数据:3,分析记录可行区间--------------------
        			        	int num=0;
        			        	if(ladar_bin[0]==0)	startcount[num]=0;
        			        	for(int i=1;i<361;i++)
        			        	{
        			        		if(ladar_bin[i-1]==0 && ladar_bin[i]==1)
        			        		{
        			        			endcount[num]=i-1;
        			        			num++;
        			        		}
        			        		else if(ladar_bin[i-1]==1 && ladar_bin[i]==0)
        			        		{
        			        			startcount[num]=i;
        			        		}
        			        		else if(num>=100)
        			        		{
        			        			break;
        			        		}
        			        	}
        			        	if(ladar_bin[360]==0)
        			        	{
        			        		endcount[num]=360;
        			        		num++;
        			        	}
        			        	numcount=num;//没有必要+1

        			        	for(int i=0;i<numcount;i++)//全部是弧度
        			        	{
        			        		startangle[i]=(startcount[i]/2.0)*(Math.PI/180)+Math.atan(carwidth/ladar[startcount[i]]);
        			        		endangle[i]=(endcount[i]/2.0)*(Math.PI/180)-Math.atan(carwidth/ladar[endcount[i]]);
        			        	}

        			        	//--------------------计算航向偏角，并转到极坐标系下（得到的角度是弧度）【经测试，数据正确】--------------------
        			        	Vector3d velocity = getVelocity(); //获取速度
        						Vector2d direct = new Vector2d(velocity.z, velocity.x); //前进的方向向量

        						getCoords(nowcoord);
        						Vector2d pos = new Vector2d(nowcoord.z, nowcoord.x);
        						Vector2d toGoal = new Vector2d((goalcoord.x - pos.x), (goalcoord.y - pos.y));
        						System.out.println("当前目标偏离航向："+goaltodirect(toGoal,direct)*(180/Math.PI));//########调试显示########
        						double goal_direct = goaltodirect(toGoal,direct)+Math.PI/2;
        			        	//--------------------分析比较得到最优转角--------------------
        						bestnum=-1;//清空bestnumSystem.out.println("当前雷达动态阈值"+m_threshold);
        						System.out.println("\n当前雷达动态阈值："+m_threshold+"\n潜在可行区域块数："+numcount);//########调试显示########
        						minangle=Math.PI;//每次清空为pi
        						for(int i=0;i<numcount;i++)
        						{
        							//########调试显示########
        							System.out.println("编号"+i+"区块"+"，起线角："+startangle[i]*(180/Math.PI)+"，终线角："+endangle[i]*(180/Math.PI)+"，中线角："+(endangle[i]+startangle[i])/2.0*(180/Math.PI));

        							if(startangle[i]<endangle[i])//潜在可行区域确实可行
        							{System.out.println("编号"+i+"区块实际可行！");//########调试显示########
        								//尝试该可行区域是否直达目标，如果是直接跳出，驶向目标
	             						if(startangle[i]<=goal_direct && goal_direct<=endangle[i])//如果恰好落在可行区域内
	    								{System.out.println("编号"+i+"区块直达目标！");//########调试显示########
	    									bestturn = goal_direct-Math.PI/2;//直接驶向目标
	    									bestnum=i;
	    									break;
	    								}
	             						else
	             							System.out.println("编号"+i+"区块不可直达目标！");//########调试显示########

        								double errorangle1 = Math.abs( startangle[i]-goal_direct );
        								if(errorangle1< minangle)
        								{
        									minangle=errorangle1;
        									bestturn=startangle[i]-Math.PI/2;
        									bestnum=i;
        								}
        								double errorangle2 = Math.abs( endangle[i]-goal_direct );
        								if(errorangle2< minangle)
        								{
        									minangle=errorangle2;
        									bestturn=endangle[i]-Math.PI/2;
        									bestnum=i;
        								}
        							}
        							else
        								System.out.println("编号"+i+"区块实际不可行！");//########调试显示########

        						}
        						if(bestnum == -1)
        						{
        							System.out.println("出现全封闭的情况！Error#################################################");

        						}
        						else
        						{
            						//########调试显示########
            						System.out.println("\n决定区块："+bestnum+"，决定转向："+bestturn*(180/Math.PI));
            						System.out.println("转向后目标偏离航向："+(goal_direct-Math.PI/2-bestturn)*(180/Math.PI));
        						}

        			//++++++++行动
        					setRotationalVelocity(bestturn);
        					setTranslationalVelocity(0.1);
            	}
        	}
        }

        /*计算到目标的距离，目前以此为停止条件
         *
         *
         */
		public double distanceGoal()
        {
			Point3d currentPos = new Point3d();
			getCoords(currentPos); //当前坐标
			Point3d goalPos = new Point3d(goal3d.x, goal3d.y, goal3d.z);

			return currentPos.distance(goalPos);

        }

       /*
        *获取激光雷达数据
        *
        *
        *
        *
        *
        *
        *
        *
        */
        //public double[] getLaserRader()
        //{

        //}
        /*机器人有720个sonar传感器，采用前361个模拟激光雷达。从
         *正左方向起顺时针从0开始编号，激光雷达的标号范围0~360，
         *此函数转换激光雷达编号为sonar编号。
         *
         *[in]      n   表示激光雷达编号
         *[return]  转换为的sounar传感器的编号
         */
        public int getControlcode(int n)
        {
        	if(n>=180 && n<=360)
        		return n-180;
        	else if(n>=0 && n<=179)
        		return n+540;
        	else
        	{
        		System.out.println("转换异常");
        		return -1;
        	}
        }
        public Vector3d getVelocity()
		{
			return this.linearVelocity; //线速度
		}
        //==================================以下三个函数用于计算目标偏航向的角度=================================
        public double goaltodirect(Vector2d togoal,Vector2d direct)
        {
        	double pianangle=angletoX(togoal)-angletoX(direct);
        	if( pianangle>Math.PI  )
        		return pianangle-2*Math.PI;
        	else if(pianangle<-Math.PI)
        		return pianangle+2*Math.PI;
        	else
        		return pianangle;
        }
        public  int getQuadrant(Vector2d vector) //识别象限区
    	{
    		double x = vector.x;
    		double y = vector.y;
    		if((x>0 && y>0)||(x>0 && y==0))
    			return 1;
    		else if((x<0 && y>0)||(x==0 && y>0))
    			return 2;
    		else if((x<0 && y<0)||(x<0 && y==0))
    			return 3;
    		else if((x>0 && y<0)||(x==0 && y<0))
    			return 4;
    		else
    		{
    			System.out.println("（0，0）无法识别象限区！");
    			return -1;
    		}
    	}
        public  double angletoX(Vector2d vector)
        {
        	switch(getQuadrant(vector))
        	{
        	case 1:return vector.angle(new Vector2d(1,0))+(Math.PI/2)*0;
        	case 2:return vector.angle(new Vector2d(0,1))+(Math.PI/2)*1;
        	case 3:return vector.angle(new Vector2d(-1,0))+(Math.PI/2)*2;
        	case 4:return vector.angle(new Vector2d(0,-1))+(Math.PI/2)*3;
        	default:
        		System.out.println("处理为面z正向");
        		return Math.PI/2;
        	}
        }

    }

    /** Describe the environement */
    static public class MyEnv extends EnvironmentDescription {
        public MyEnv() {
            light1IsOn = true;
            light2IsOn = false;
            Wall w1 = new Wall(new Vector3d(9, 0, 0), 19, 1, this);
            w1.rotate90(1);
            add(w1);
            Wall w2 = new Wall(new Vector3d(-9, 0, 0), 19, 2, this);
            w2.rotate90(1);
            add(w2);
            Wall w3 = new Wall(new Vector3d(0, 0, 9), 19, 1, this);
            add(w3);
            Wall w4 = new Wall(new Vector3d(0, 0, 5), 15, 2, this);
            add(w4);
            Wall w5 = new Wall(new Vector3d(0, 0, -9), 19, 2, this);
            add(w5);
            Box b1 = new Box(new Vector3d(-3, 0, -3), new Vector3f(2f, 2f, 2f),this);
            add(b1);
            Box b2 = new Box(new Vector3d(3, 0, 6), new Vector3f(3f, 3f, 3f),this);
            add(b2);
            Box b3 = new Box(new Vector3d(7, 0, 2), new Vector3f(2.8f, 3f, 2.8f),this);
            add(b3);
            Box b4 = new Box(new Vector3d(0, 0, 0), new Vector3f(3f, 3f, 3f),this);
            add(b4);
            add(new Arch(new Vector3d(3, 0, -3), this));
            add(new Robot(new Vector3d(-8, 0, -8), "robot 1"));

        }
    }

    public static void main(String[] args) {
        // request antialising
        System.setProperty("j3d.implicitAntialiasing", "true");
        // create Simbad instance with given environment
        Simbad frame = new Simbad(new MyEnv(), false);
    }

}
