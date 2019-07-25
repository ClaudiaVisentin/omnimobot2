#include <LSData.hpp>
#include <CommandURG04LX.hpp>

#include <iostream>



int main(int argc, char *argv[]) 
{

	std::cout << "LSData test started..." << std::endl;
	
	int start = 370;
	int end = 450;
	
	
	ScanCommand command;
	command.setStartValue(start);
	command.setEndValue(end);
	
	std::cout << "Command: " << command;
	
	LSData data(start,end);
	
	
	// 46 - 600
//	char buffer[] = "MD0046060000001\n00P\n\nMD0046060000000\n99b\n0dWTo\n0:[0:Y0:Y0:Y0:U0:U0:Q0:G0:G0:G0:E0:A0:@0:>0:>0:<0:;0:<0:<0:<0:;00\n:70:10:009m09m09l09l09j09j09I09A09A09D09D09V0:40:E0:T0:k0;:0;Q0;]\n^0;^0<40<H0<l0=@0=?0>90>E0@N0@N0@N0@N0@I0@E0@C0@E0@C0@C0@A0@A0@>W\n0@>0@<0@<0@<0@>0@<0@;0@;0@;0@90@80@80@80@80@80@80@:0@80@90@80@80Q\n@80@80@80@60@60@60@60@20@20@60@:0@<0@>0@?0@>0@>0@90@90@90@90@90@F\n90@90@90@90@?0@?0@@0@@0@A0@A0@E0@H0@H0@E0@E0@I0@J0@K0@M0@Q0@Q0@UE\n0@U0@U0@T0?c0?I0>d0=i0<V0;00:o0:o0:l0:o0:n0:n0;00;10;10;10:a0:S0E\n:S0:S0:Z0:[0::0:^0;i0<A0<E0<G0<K0@[0A^0Ad0Ag0Ah0Am0An0B10B50B80B0\nA0BE0BF0BG0BJ0BQ0BQ0BU0B[0B_0B_0B[0Bf0Be0Be0Bg0Bh0C40C@0CA0CF0CFW\n0CF0CF0BN0A509709709709709108m08g08m08g08c08c08c08[08Y08T08T08S02\n8N08M08K08J08J08G08A08808808207o07o07n07k07i07n07c07b07b07]07]077\n[07[07[07[07[07U07V07V07V07S07S07S07S07S07Q07Q07O07L07L07K07I07GE\n07C07@07@07@07=07=07=07807:07807807807707407006o06n06m06m06k06k0I\n6e06k06g06e06g06g06f06f06f06f06f06e06e06k07H08>08I08I08J08I08F08a\n708708708>08>08>07j07j07j08@08E08V08g09409I09T0:20;[0;n0<20<50<9W\n0<90<90<;0<90<60<60<:0<90<90<90<60<60<60<70<:0<=0<=0<:0<>0<>0<>04\n<>0<>0<;0<;0<90<;0<;0<90<50;e0;e0;e0;e0:e0:<0:X07c07R07O07?07?07^\n?07E07H07P07Q07R07R07R07R07S07U07U07U07O07O07O07I07<07?07<07<07=P\n07=07907=07D0:P0<10<70<>0<>0<C0<E0<G0<J0<J0<N0<N0<N0<H0<N0<K0<P0O\n<U0<V0<X0<_0<_0<`0<a0<a0<d0<d0<d09109109<09a09l09l09n09n09n0:30:B\n30:309i09i09i09i09i09b09b09b0<l0=Z0=Z0=Z0=Z0=Z0=V0=V00704X04X04WF\n04W04W04W04W04W04T04P04O04>04>04>04@04@04@04C04C04F04K04U04W04Z0Q\n4Z04[04b04b04i04j05609A0000070000000000000000000000000000000000B2\n90Ae0?:0>?0>:0>:0>?0>?0>:0>90>90>90>C0>M0>R0>Z0>c0>j0?;0?B0?I0?RM\n0?]0?l0@10@;0@H0000000000=J0=@0<o0<o0<o0=60=@0000000000000000000`\n000000000000000I10HH0:=0:90:90:90:90:90:;0:;0E]0E]0E]0D^0D^0D^0E;\n9i";
	
	// 384
// 	char buffer[] = "MD0384038400001\n00P\n\nMD0384038400000\n99b\nH:6mU\n03nA";
	
	 // 370 - 450
char buffer[] = "MD0370045000001\n00P\n\nMD0370045000000\n99b\nO7e=\n03T03T03T03Z03Z03\\03\\03\\03[03[03[03\\03\\03\\03\\03\\03_03a03a03e03e0K\n3d03c03c03_03Z03W03Z03Z03Z03]03]03]03c03c03f03f03h03h03m03h03h03@\nh03h03h03h03i03i03h03k03k03k03k03m03o03o04004003o04104003o040041f\n04204404604604204204004003o03o03m03k03k03k03h03h03gd";
	
	int bufferSize = 0;
	while(buffer[bufferSize] != 0){
		bufferSize++;
	}
	
	std::cout << "bufferSize: " << bufferSize << std::endl;
	
	if(!command.check(buffer)){
		std::cout << "wrong results " << std::endl;
		return 0;
	}
	
	data.decode(buffer,bufferSize);
	
	eeros::math::Matrix<682,2> distancAndAngle = data.getData();
	
	std::cout << "scannData : " << distancAndAngle(0 ,0 )  << " angle: "<< distancAndAngle(0 ,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(1 ,0 )  << " angle: "<< distancAndAngle(1 ,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(2 ,0 )  << " angle: "<< distancAndAngle(2 ,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(3 ,0 )  << " angle: "<< distancAndAngle(3 ,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(4 ,0 )  << " angle: "<< distancAndAngle(4 ,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(5 ,0 )  << " angle: "<< distancAndAngle(5 ,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(6 ,0 )  << " angle: "<< distancAndAngle(6 ,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(7 ,0 )  << " angle: "<< distancAndAngle(7 ,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(8 ,0 )  << " angle: "<< distancAndAngle(8 ,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(9 ,0 )  << " angle: "<< distancAndAngle(9 ,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(10,0 )  << " angle: "<< distancAndAngle(10,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(11,0 )  << " angle: "<< distancAndAngle(11,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(12,0 )  << " angle: "<< distancAndAngle(12,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(13,0 )  << " angle: "<< distancAndAngle(13,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(14,0 )  << " angle: "<< distancAndAngle(14,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(15,0 )  << " angle: "<< distancAndAngle(15,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(16,0 )  << " angle: "<< distancAndAngle(16,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(17,0 )  << " angle: "<< distancAndAngle(17,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(18,0 )  << " angle: "<< distancAndAngle(18,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(19,0 )  << " angle: "<< distancAndAngle(19,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(20,0 )  << " angle: "<< distancAndAngle(20,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(21,0 )  << " angle: "<< distancAndAngle(21,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(22,0 )  << " angle: "<< distancAndAngle(22,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(23,0 )  << " angle: "<< distancAndAngle(23,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(24,0 )  << " angle: "<< distancAndAngle(24,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(25,0 )  << " angle: "<< distancAndAngle(25,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(26,0 )  << " angle: "<< distancAndAngle(26,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(27,0 )  << " angle: "<< distancAndAngle(27,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(28,0 )  << " angle: "<< distancAndAngle(28,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(29,0 )  << " angle: "<< distancAndAngle(29,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(30,0 )  << " angle: "<< distancAndAngle(30,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(31,0 )  << " angle: "<< distancAndAngle(31,1)  << std::endl;
	std::cout << "scannData : " << distancAndAngle(32,0 )  << " angle: "<< distancAndAngle(32,1)  << std::endl;
	                                               
                                                   
	                                               
	
	std::cout << "LSData test ended..." << std::endl;
	
	return 0;
}