package VFHsimulation;
/***********************************************************************
 * Version:1.3.1
 * Author :yin
 * Date   :2016-8-14
 * About  :VFH�ķ����������720����������ֻʹ��ǰ���361���������״ﴫ����Ϊ����
 * ���顣��һ�������ܿ���Ŀ���һ������Ϊ����ת��ladar�����������޸�Ϊ8��
 * ################################ FIX ################################
 * Fix Version:1.0
 * Fix Author :yin
 * Fix Date   :2016-8-13
 * Fix About  :��ɻ����ĳ����ܣ�������һЩ���õĴ��󣬲�������ͨ���������������
 * �Ŀ�ȡ�����������ײ���⡣
 * ################################ FIX ################################
 * Fix Version:1.1
 * Fix Author :yin
 * Fix Date   :2016-8-14
 * Fix About  :�������������Ŀ�ȣ�������һ����ȫ�뾶����С���еķ���Ƕȣ�����
 * ����������������Ŀ��ĺ��򣬾�ֱ������Ŀ�ꡣ������ǣ��������Ŀ�꺽��Ŀ���
 * ����߽硣ʵ��֤����Ȼ��Ȼ������ײ���⣬���ǹ滮��·������һ���汾���������߸���v��
 * �������Ż������ȼ����ȵķ������š�
 * ################################ FIX ################################
 * Fix Version:1.2
 * Fix Author :yin
 * Fix Date   :2016-8-18
 * Fix About  :����һ���汾���Ż������ԣ��޸�������ݣ�����Ҫ�ĵ��Բ�����ʾ������
 * �����˶�̬��ֵ�Ĺ��ܡ��˰汾��������ʦ�ֵİ汾�����������˼�С��������Ƕȵ��Ż���
 * ������Ϊ�ΰ汾�Ĵ��µ����ڣ�1����ֵ��̬��ֵ�������ȫ������⣩2����������������
 * �ķ����������ܿ���Ŀ�귽λ�������ȫ�������⣩������һ�汾�б�������һ����С����
 * ����Ƕȵ��Ż������˰汾��Ϊ���˳���Ŀ���գ�������Ƶ¼�Ƶİ汾(����5�ף������˰�
 * ��0.3��)��
 * ################################ FIX ################################
 * Fix Version:1.2.1
 * Fix Author :yin
 * Fix Date   :2016-8-18 night
 * Fix About  :�޸���0֡�������ٶȷ���Ĵ���Ĭ�ϻ�����0֡��z����
 * ################################ FIX ################################
 * Fix Version:1.3
 * Fix Author :yin
 * Fix Date   :2016-8-19 
 * Fix About  :
 * 1�����Ŀ�������̷�Χ�ڿɼ�����Ŀ��Ϊ�պÿɼ��������Ŀ�긽����
 * �ϰ��������
 * 2��Ԥ����ȫ����߼�
 * ################################ FIX ################################
 * Fix Version:1.3.1
 * Fix Author :yin
 * Fix Date   :2016-8-19 
 * Fix About  :
 * 1���޸���Ŀ�����������Ŀ��ɼ�������
 * 2��֤��������Ŀ���߼�����Ҫ��
 * 3���������һ�ֶ�̬��ֵ�ļ��㷽����Ŀǰ�������ߵ����𲻴�
 * 4����������Ĵ�������˶���ȫ��գ��������ײ֮���ì�ܣ�
 * 5��������·�����湦��
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

        
        Vector2d goalcoord = new Vector2d(8,8); //�洢Ŀ���
        Vector3d goal3d = new Vector3d(8, 0,8);
        double carwidth=0.3;
        
        //--------------------���㺽��ƫ����--------------------
        Point3d nowcoord = new Point3d();//�˿̵�����
        double errorangle;//���㺽��ƫ��
        
        //--------------------����ladar������--------------------
        double ladar[] = new double[361];//�洢������ladr����
        int ladar_bin[] = new int[361];//�洢������ladr�Ķ�ֵ����
        double ladarmin,ladarmax,ladarmid;
        
        float liangcheng=5f;//����
        double m_threshold;//������̬��ֵ�����뻷�������й�
        
        int numcount = 0;//��¼��������
        int startcount[] = new int[400];//��¼���������
        int endcount[] = new int[400];//��¼�����յ���
        double startangle[] = new double[400];//��¼�������߻���
        double endangle[] = new double[400];//��¼�������߻���
        double midangle[] = new double[400];//��¼�������߻���
              
        double minangle;
        double bestturn;//����ת��Ļ���
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
        		saveTrajectory(20);
            	if(getCounter()%20==0)//Ϊ��ֹ���һζ���ÿ10֡��һ��
            	{System.out.println("-----------------------------------------------------------------");
                    //++++++++˼��
        			        	//--------------------����ladar����:1,��ȡ���������ݵ�����--------------------
        			        	for(int n=0;n<361;n++)
        			        	{
        			        		ladar[n]=sonars.getMeasurement(getControlcode(n));//��ȡ����
        			        		if(ladar[n]==Double.POSITIVE_INFINITY)//����������̣�����Ϊ����
        			        			ladar[n]=liangcheng;
        			        	}
        		        		/*if( getCounter()==0 );//����radar�����Ƿ���ȷ������ĳʱ�̵�radar���ı����ڴ�matlab�д�
        		        		{
        		        			File file = new File("d:/1.txt");  
        		        		        try  
        		        		        {  
        		        		            FileWriter fw = new FileWriter(file);  
        		        		            BufferedWriter bw = new BufferedWriter (fw);
        		        		            for(int i=0;i<361;i++)
        		        		            {
        		        		            	fw.write(Double.toString(ladar[i])); 
        		        		            	fw.write("\n");
        		        		            }
        		        		            fw.close();  
        		        		              
        		        		        }  
        		        		        catch (Exception e)  
        		        		        {  
        		        		            e.printStackTrace();  
        		        		        } 
        		        		}*/
        			        	//--------------------����ladar����:2,��̬��ֵ����ɶ�ֵ����--------------------
        			        	
        			        	/*ladarmin=ladar[0];//�ҵľ�ֵ���㷽��
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
        			        	
        			        	/*double sum=0;//��̬��ֵ��������
        			        	for(int i=0;i<61;i++)//��ȡ6�ı����ĵ�,���ȡ��ֵ
        			        		sum=sum+ladar[i*6];
        			        	m_threshold=sum/61;*/
        			        	
        			        	double sum=0;
        			        	  for(int i=0;i<361;i++)//���е��ֵ
        			        		sum=sum+ladar[i];
        			        	m_threshold=sum/361;
        			        	
        			        	if(m_threshold>=liangcheng)
        			        		m_threshold = liangcheng;
	
        			        	if(distanceGoal()<=m_threshold)//���Ŀ���ڿɼ���Χ�ڣ���Ŀ��Ϊ�պÿɼ�������Ŀ�꡿
        			        		m_threshold=distanceGoal();
        			        	
        			        		
    			        		for(int i=0;i<361;i++)
    			        		{
    			        			if(ladar[i]>=m_threshold)	ladar_bin[i]=0;
    			        			else						ladar_bin[i]=1;
    			        		}
    			        		//��ֵ��ֵ������£������ܳ���ȫ��գ����������߷��������˴���ȫ���ŵ��߼�
    			        		 		
        		        		/*if( getCounter()==0 );//����radar_bin�����Ƿ���ȷ������ĳʱ�̵�radar_bin���ı����ڴ�matlab�д�
        		        		{
        		        			File file = new File("d:/1.txt");  
        		        		        try  
        		        		        {  
        		        		            FileWriter fw = new FileWriter(file);  
        		        		            BufferedWriter bw = new BufferedWriter (fw);
        		        		            for(int i=0;i<361;i++)
        		        		            {
        		        		            	fw.write(Integer.toString(ladar_bin[i])); 
        		        		            	fw.write("\n");
        		        		            }
        		        		            fw.close();  
        		        		              
        		        		        }  
        		        		        catch (Exception e)  
        		        		        {  
        		        		            e.printStackTrace();  
        		        		        } 
        		        		}*/
        			        	//--------------------����ladar����:3,������¼��������--------------------
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
        			        	numcount=num;//û�б�Ҫ+1        			        	
        			        	
        			        	for(int i=0;i<numcount;i++)//ȫ���ǻ���
        			        	{
        			        		startangle[i]=(startcount[i]/2.0)*(Math.PI/180)+Math.atan(carwidth/ladar[startcount[i]]);
        			        		endangle[i]=(endcount[i]/2.0)*(Math.PI/180)-Math.atan(carwidth/ladar[endcount[i]]);
        			        	}
        						
        			        	//--------------------���㺽��ƫ�ǣ���ת��������ϵ�£��õ��ĽǶ��ǻ��ȣ��������ԣ�������ȷ��--------------------
        			        	Vector3d velocity = getVelocity(); //��ȡ�ٶ� 
        						Vector2d direct = new Vector2d(velocity.z, velocity.x); //ǰ���ķ�������
        						
        						getCoords(nowcoord); 
        						Vector2d pos = new Vector2d(nowcoord.z, nowcoord.x); 
        						Vector2d toGoal = new Vector2d((goalcoord.x - pos.x), (goalcoord.y - pos.y));
        						System.out.println("��ǰĿ��ƫ�뺽��"+goaltodirect(toGoal,direct)*(180/Math.PI));//########������ʾ########	
        						double goal_direct = goaltodirect(toGoal,direct)+Math.PI/2;
        			        	//--------------------�����Ƚϵõ�����ת��--------------------
        						bestnum=-1;//���bestnumSystem.out.println("��ǰ�״ﶯ̬��ֵ"+m_threshold);       						
        						System.out.println("\n��ǰ�״ﶯ̬��ֵ��"+m_threshold+"\nǱ�ڿ������������"+numcount);//########������ʾ########
        						minangle=Math.PI;//ÿ�����Ϊpi
        						for(int i=0;i<numcount;i++)
        						{
        							//########������ʾ########
        							System.out.println("���"+i+"����"+"�����߽ǣ�"+startangle[i]*(180/Math.PI)+"�����߽ǣ�"+endangle[i]*(180/Math.PI)+"�����߽ǣ�"+(endangle[i]+startangle[i])/2.0*(180/Math.PI));	
        							
        							if(startangle[i]<endangle[i])//Ǳ�ڿ�������ȷʵ����
        							{System.out.println("���"+i+"����ʵ�ʿ��У�");//########������ʾ########
        								//���Ըÿ��������Ƿ�ֱ��Ŀ�꣬�����ֱ��������ʻ��Ŀ��
	             						if(startangle[i]<=goal_direct && goal_direct<=endangle[i])//���ǡ�����ڿ���������
	    								{System.out.println("���"+i+"����ֱ��Ŀ�꣡");//########������ʾ########
	    									bestturn = goal_direct-Math.PI/2;//ֱ��ʻ��Ŀ��
	    									bestnum=i;
	    									break;
	    								}
	             						else
	             							System.out.println("���"+i+"���鲻��ֱ��Ŀ�꣡");//########������ʾ########
        							
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
        								System.out.println("���"+i+"����ʵ�ʲ����У�");//########������ʾ########
        								
        						}
        						if(bestnum == -1)
        						{
        							System.out.println("����ȫ��յ������Error#################################################");

        						}
        						else
        						{
            						//########������ʾ########
            						System.out.println("\n�������飺"+bestnum+"������ת��"+bestturn*(180/Math.PI));
            						System.out.println("ת���Ŀ��ƫ�뺽��"+(goal_direct-Math.PI/2-bestturn)*(180/Math.PI));	
        						}
			 
        			//++++++++�ж�
        					setRotationalVelocity(bestturn);
        					setTranslationalVelocity(0.1);
            	}
        	}
        }
		public double distanceGoal() //����Ƿ񵽴�Ŀ�ĵ�
        { 
			Point3d currentPos = new Point3d(); 
			getCoords(currentPos); //��ǰ����
			Point3d goalPos = new Point3d(goal3d.x, goal3d.y, goal3d.z); 
		
			return currentPos.distance(goalPos); 
	
        } 
        public int getControlcode(int n)
        {
        	if(n>=180 && n<=360)
        		return n-180;
        	else if(n>=0 && n<=179)
        		return n+540;
        	else 
        	{
        		System.out.println("ת���쳣");
        		return -1;
        	}
        }
        public Vector3d getVelocity() 
		{ 
			return this.linearVelocity; //���ٶ�
		}
        //==================================���������������ڼ���Ŀ��ƫ����ĽǶ�=================================
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
        public  int getQuadrant(Vector2d vector) //ʶ��������
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
    			System.out.println("��0��0���޷�ʶ����������");
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
        		System.out.println("����Ϊ��z����");
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