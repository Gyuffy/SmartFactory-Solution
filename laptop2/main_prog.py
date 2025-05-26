from robodk import robolink  # RoboDK API
from robodk import robomath  # Robot toolbox

from robodk import *  # RoboDK API
from robolink import *  # Robot toolbox


def main_process():
    RDK = robolink.Robolink()
    rack1 = RDK.Item("R_FrontPanel", 2)
    rack2 = RDK.Item("R_Screen", 2)
    rack3 = RDK.Item("R_Frame", 2)
    rack4 = RDK.Item("R_Back", 2)

    F1 = RDK.Item("FrontPanel_T", 3)
    F2 = RDK.Item("Screen_T", 3)
    F3 = RDK.Item("Frame_T", 3)
    F4 = RDK.Item("Back_T", 3)

    TVpart1_1 = RDK.Item("TVFrontPanel01", 5)
    TVpart1_2 = RDK.Item("TVFrontPanel02", 5)
    TVpart1_3 = RDK.Item("TVFrontPanel03", 5)
    TVpart1_4 = RDK.Item("TVFrontPanel04", 5)
    TVpart1_5 = RDK.Item("TVFrontPanel05", 5)
    TVpart1_6 = RDK.Item("TVFrontPanel06", 5)
    TVpart1_7 = RDK.Item("TVFrontPanel07", 5)
    TVpart1_8 = RDK.Item("TVFrontPanel08", 5)
    TVpart1_9 = RDK.Item("TVFrontPanel09", 5)
    TVpart1_10 = RDK.Item("TVFrontPanel10", 5)

    TVpart1 = [
        "TVFrontPanel10",
        "TVFrontPanel09",
        "TVFrontPanel08",
        "TVFrontPanel07",
        "TVFrontPanel06",
        "TVFrontPanel05",
        "TVFrontPanel04",
        "TVFrontPanel03",
        "TVFrontPanel02",
        "TVFrontPanel01",
    ]
    TVpart1_All = [
        TVpart1_10,
        TVpart1_9,
        TVpart1_8,
        TVpart1_7,
        TVpart1_6,
        TVpart1_5,
        TVpart1_4,
        TVpart1_3,
        TVpart1_2,
        TVpart1_1,
    ]

    TVpart2_1 = RDK.Item("TVScreen01", 5)
    TVpart2_2 = RDK.Item("TVScreen02", 5)
    TVpart2_3 = RDK.Item("TVScreen03", 5)
    TVpart2_4 = RDK.Item("TVScreen04", 5)
    TVpart2_5 = RDK.Item("TVScreen05", 5)
    TVpart2_6 = RDK.Item("TVScreen06", 5)
    TVpart2_7 = RDK.Item("TVScreen07", 5)
    TVpart2_8 = RDK.Item("TVScreen08", 5)
    TVpart2_9 = RDK.Item("TVScreen09", 5)
    TVpart2_10 = RDK.Item("TVScreen10", 5)
    TVpart2 = [
        "TVScreen10",
        "TVScreen09",
        "TVScreen08",
        "TVScreen07",
        "TVScreen06",
        "TVScreen05",
        "TVScreen04",
        "TVScreen03",
        "TVScreen02",
        "TVScreen01",
    ]
    TVpart2_All = [
        TVpart2_10,
        TVpart2_9,
        TVpart2_8,
        TVpart2_7,
        TVpart2_6,
        TVpart2_5,
        TVpart2_4,
        TVpart2_3,
        TVpart2_2,
        TVpart2_1,
    ]

    TVpart3_1 = RDK.Item("TVFrame1")

    TVpart4_1 = RDK.Item("BackPanel1")

    R1 = RDK.Item("ABB1", 2)
    RT1 = RDK.Item("ABB1_T", 4)
    R2 = RDK.Item("ABB2", 2)
    RT2 = RDK.Item("ABB2_T", 4)
    R3 = RDK.Item("ABB3", 2)
    RT3 = RDK.Item("ABB3_T", 4)
    R4 = RDK.Item("Fanuc1", 2)
    RT4 = RDK.Item("Fanuc1_T", 4)

    CON1 = RDK.Item("CON1", 2)
    CONTF1 = RDK.Item("CON1_TF", 3)
    CON2 = RDK.Item("CON2", 2)
    CONTF2 = RDK.Item("CON2_TF", 3)
    CON3 = RDK.Item("CON3", 2)
    CONTF3 = RDK.Item("CON3_TF", 3)
    CON4 = RDK.Item("CON4", 2)
    CONTF4 = RDK.Item("CON4_TF", 3)
    CON5 = RDK.Item("CON5", 2)
    CONTF5 = RDK.Item("CON5_TF", 3)

    GUIDE = RDK.Item("TV_Guide", 5)

    box1 = RDK.Item("BOX_Open01", 5)
    box2 = RDK.Item("BOX_Open02", 5)
    box3 = RDK.Item("BOX_Open03", 5)
    box4 = RDK.Item("BOX_Open04", 5)
    box5 = RDK.Item("BOX_Open05", 5)
    box6 = RDK.Item("BOX_Open06", 5)
    box7 = RDK.Item("BOX_Open07", 5)
    box8 = RDK.Item("BOX_Open08", 5)
    box9 = RDK.Item("BOX_Open09", 5)
    box10 = RDK.Item("BOX_Open10", 5)
    box_c1 = RDK.Item("BOX_Close01", 5)
    box_c2 = RDK.Item("BOX_Close02", 5)
    box_c3 = RDK.Item("BOX_Close03", 5)
    box_c4 = RDK.Item("BOX_Close04", 5)
    box_c5 = RDK.Item("BOX_Close05", 5)
    box_c6 = RDK.Item("BOX_Close06", 5)
    box_c7 = RDK.Item("BOX_Close07", 5)
    box_c8 = RDK.Item("BOX_Close08", 5)
    box_c9 = RDK.Item("BOX_Close09", 5)
    box_c10 = RDK.Item("BOX_Close10", 5)
    BOX_F = RDK.Item("BOX_F", 3)

    boxo_All = [box10, box9, box8, box7, box6, box5, box4, box3, box2, box1]
    boxc_All = [
        box_c10,
        box_c9,
        box_c8,
        box_c7,
        box_c6,
        box_c5,
        box_c4,
        box_c3,
        box_c2,
        box_c1,
    ]
    boxo = [
        "BOX_Open10",
        "BOX_Open09",
        "BOX_Open08",
        "BOX_Open07",
        "BOX_Open06",
        "BOX_Open05",
        "BOX_Open04",
        "BOX_Open03",
        "BOX_Open02",
        "BOX_Open01",
    ]
    boxc = [
        "BOX_Close10",
        "BOX_Close09",
        "BOX_Close08",
        "BOX_Close07",
        "BOX_Close06",
        "BOX_Close05",
        "BOX_Close04",
        "BOX_Close03",
        "BOX_Close02",
        "BOX_Close01",
    ]

    PCB1 = RDK.Item("PART01", 5)
    PCB2 = RDK.Item("PART02", 5)
    PCB3 = RDK.Item("PART03", 5)
    PCB4 = RDK.Item("PART04", 5)
    PCB5 = RDK.Item("PART05", 5)
    PCB6 = RDK.Item("PART06", 5)
    PCB7 = RDK.Item("PART07", 5)
    PCB1_TF = RDK.Item("PCB1_TF", 3)  # 1,3,4,7
    PCB2_TF = RDK.Item("PCB2_TF", 3)
    PCB1_R = RDK.Item("PCB1", 2)
    PCB2_R = RDK.Item("PCB2", 2)

    Foam_TF = RDK.Item("Foam_TF", 3)
    Foam1 = RDK.Item("Foam_D", 5)
    Foam2 = RDK.Item("Foam_U", 5)
    Foam_R = RDK.Item("Foam", 2)

    Delta1 = RDK.Item("Delta1", 2)
    Delta1_T = RDK.Item("Delta1_T", 4)
    Delta2 = RDK.Item("Delta2", 2)
    Delta2_T = RDK.Item("Delta2_T", 4)

    AGV1 = RDK.Item("AGV1", 2)
    AGV1_TF = RDK.Item("AGV1_TF", 3)

    def job1(num):

        rack1.MoveJ([0.000000, 0.000000])
        rack1.MoveJ([-359.000000, 0.000000])

        num2 = num * 30

        pos1 = Mat(
            [
                [0.55, -0.28, 0.77, 1134.00],
                [-0.82, -0.19, 0.52, -380],
                [0, -1, -0.342020, 170.00 - num2],
                [0, 0, 0, 1],
            ]
        )
        R1.MoveL(pos1)
        RT1.AttachClosest(TVpart1[num], 1000)

        R1.MoveL(
            Mat(
                [
                    [0.55, -0.28, 0.77, 1134.00],
                    [-0.82, -0.19, 0.52, -380],
                    [0, -1, -0.342020, 600],
                    [0, 0, 0, 1],
                ]
            )
        )
        R1.MoveJ(
            [-72.655210, 31.996484, 52.526766, -117.032282, -85.624136, -54.779882]
        )  # 놓기전 상승위치
        R1.MoveJ(
            [-72.655191, 64.934441, 76.995869, -113.211109, -75.099271, -77.224687]
        )  # 컨베이어 놓는 위치
        RT1.DetachAll(CONTF1)
        TVpart1_All[num].setPose(
            Mat([[0, -1, 0, -30], [1, 0, 0, 795], [0, 0, 1, -160], [0, 0, 0, 1]])
        )

        R1.MoveJ(
            [-72.655210, 31.996484, 52.526766, -117.032282, -85.624136, -54.779882]
        )  # 놓기전 상승위치

    def job2():
        CON1.MoveJ([-2200.000000])

    def job3(num):
        rack1.MoveJ([-3250.000000, 0.000000])
        rack2.MoveJ([-0.000000, -2100.000000])
        rack2.MoveJ([0.000000, 0.000000])
        rack2.MoveJ([-359.00000, 0.000000])

        num2 = num * 20

        pos1 = Mat(
            [
                [0.55, -0.28, 0.77, 1134.00],
                [-0.82, -0.19, 0.52, -380],
                [0, -1, -0.342020, 70.00 - num2],
                [0, 0, 0, 1],
            ]
        )
        R1.MoveL(pos1)

        RT1.AttachClosest(TVpart2[num], 1000)

        R1.MoveL(
            Mat(
                [
                    [0.55, -0.28, 0.77, 1134.00],
                    [-0.82, -0.19, 0.52, -380],
                    [0, -1, -0.342020, 600],
                    [0, 0, 0, 1],
                ]
            )
        )
        R1.MoveJ(
            [-72.655210, 31.996484, 52.526766, -117.032282, -85.624136, -54.779882]
        )  # 놓기전 상승위치
        R1.MoveJ(
            [-72.655191, 64.934441, 76.995869, -113.211109, -75.099271, -77.224687]
        )  # 컨베이어 놓는 위치
        RT1.DetachAll(CONTF1)

        TVpart2_All[num].setPose(
            Mat([[0, -1, 0, 2200], [1, 0, 0, 795], [0, 0, 1, -150], [0, 0, 0, 1]])
        )

        R1.MoveJ(
            [-72.655210, 31.996484, 52.526766, -117.032282, -85.624136, -54.779882]
        )  # 놓기전 상승위치

    def job4():
        CON1.MoveJ([-4720.000000])

    def job5():
        # Guide pick
        R2.MoveJ(
            [-146.444968, 18.269729, 76.085701, -36.450724, -59.759104, -77.080008]
        )  # 가이드 잡기 전 위치
        R2.MoveJ(
            [-146.323246, 58.830715, 113.845030, -30.782832, -91.592627, -98.380724]
        )  # 가이드 잡기위한 위치
        RT2.AttachClosest("TV_Guide", 1000)
        R2.MoveJ(
            [-146.444968, 18.269729, 76.085701, -36.450724, -59.759104, -77.080008]
        )  # 가이드 잡기 전 위치

        R2.MoveJ(
            [-103.301352, 38.205475, 48.252689, -96.276467, -69.096794, -41.836787]
        )  # 컨베이어4 놓기 전 위치
        R2.MoveJ(
            [-103.265970, 52.293869, 61.366543, -91.111421, -68.226863, -55.889658]
        )  # 컨베이어4에 놓는 위치

        RT2.DetachAll(CONTF4)  # 가이드를 4번 컨베이어에 놓기
        GUIDE.setPose(
            Mat([[0, -1, 0, -1926.25], [0, 0, 1, 896], [-1, 0, 0, 861], [0, 0, 0, 1]])
        )  # setpos를 이용해서 가이드를 강제로 특정위치로 고정

        R2.MoveJ(
            [-103.301352, 38.205475, 48.252689, -96.276467, -69.096794, -41.836787]
        )  # 컨베이어 놓기 전 위치
        R2.MoveJ(
            [-30.989999, 47.199976, 63.679830, -78.940539, -62.649711, -62.389592]
        )  # 로봇이 초기위치로 이동(Home)

    def job6(num):
        R2.MoveJ(
            [35.743958, 46.457908, 79.800060, 65.711867, -65.572361, -99.507682]
        )  # TV부품집기위해 컨베이어 벨트1 업위치
        R2.MoveJ(
            [35.761632, 60.636576, 87.774284, 62.722530, -69.048303, -91.726177]
        )  # TV부품집는 위치
        RT2.AttachClosest(TVpart1[num], 1000)
        R2.MoveJ(
            [35.743958, 46.457908, 79.800060, 65.711867, -65.572361, -99.507682]
        )  # TV부품잡은 후 컨베이어 벨트1 업위치
        R2.MoveJ(
            [-30.989999, 47.199976, 63.679830, -78.940539, -62.649711, -62.389592]
        )  # Home
        R2.MoveJ(
            [-103.301352, 38.205475, 48.252689, -96.276467, -69.096794, -41.836787]
        )  # 컨베이어4 놓기 전 위치
        R2.MoveJ(
            [-103.268457, 51.204353, 60.557188, -91.436632, -68.246066, -55.018430]
        )  # 컨베이어 4 놓는 위치
        RT2.DetachAll(CONTF4)
        TVpart1_All[num].setPose(
            Mat([[1, 0, 0, -1862.1], [0, 1, 0, 1148.85], [0, 0, 1, 76.5], [0, 0, 0, 1]])
        )  # 위치 보정
        R2.MoveJ(
            [-103.301352, 38.205475, 48.252689, -96.276467, -69.096794, -41.836787]
        )  # 컨베이어4 놓기 전 위치
        R2.MoveJ(
            [-30.989999, 47.199976, 63.679830, -78.940539, -62.649711, -62.389592]
        )  # Home

    def job7():
        CON1.MoveJ([-2200.000000 - 4720.000000])

    def job8(num):
        R2.MoveJ(
            [35.743958, 46.457908, 79.800060, 65.711867, -65.572361, -99.507682]
        )  # TV부품집기위해 컨베이어 벨트1 업위치
        R2.MoveJ(
            [35.761632, 60.636576, 87.774284, 62.722530, -69.048303, -91.726177]
        )  # TV부품집는 위치
        RT2.AttachClosest(TVpart2[num], 1000)
        R2.MoveJ(
            [35.743958, 46.457908, 79.800060, 65.711867, -65.572361, -99.507682]
        )  # TV부품집기위해 컨베이어 벨트1 업위치
        R2.MoveJ(
            [-30.989999, 47.199976, 63.679830, -78.940539, -62.649711, -62.389592]
        )  # Home
        R2.MoveJ(
            [-103.301352, 38.205475, 48.252689, -96.276467, -69.096794, -41.836787]
        )  # 컨베이어4 놓기 전 위치
        R2.MoveJ(
            [-103.268457, 51.204353, 60.557188, -91.436632, -68.246066, -55.018430]
        )  # 컨베이어 3 놓는 위치
        RT2.DetachAll(CONTF4)
        TVpart2_All[num].setPose(
            Mat([[1, 0, 0, -1862.1], [0, 1, 0, 1148.85], [0, 0, 1, 81], [0, 0, 0, 1]])
        )
        R2.MoveJ(
            [-103.301352, 38.205475, 48.252689, -96.276467, -69.096794, -41.836787]
        )  # 컨베이어4 놓기 전 위치
        R2.MoveJ(
            [-30.989999, 47.199976, 63.679830, -78.940539, -62.649711, -62.389592]
        )  # Home

    def job9():
        CON1.setPose(Mat([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]))

    def job10():
        rack1.MoveJ([-3250.000000, -1140.000000])
        rack2.MoveJ([-3250.000000, 0.000000])

    def job11():
        TVpart3_1.setVisible(True)
        rack3.MoveJ([0.000000, -2100.000000])
        rack3.MoveJ([0.000000, 0.000000])
        rack3.MoveJ([-359.320000, 0.000000])

    def job12():
        # pickup
        R1.MoveJ(
            [-30.989999, 47.199976, 63.679830, -78.940539, -62.649711, -62.389592]
        )  # home
        R1.MoveJ([-31.034719, 61.798349, 72.753418, -74.608222, -64.747579, -72.240488])
        RT1.AttachClosest("TVFrame1", 1000)
        R1.MoveJ([-30.913124, 27.035174, 42.085122, -90.649707, -60.609735, -37.912265])
        R1.MoveJ(
            [-72.659792, 46.141127, 66.033817, -115.461384, -79.630306, -66.991431]
        )
        R1.MoveJ(
            [-72.655191, 64.934441, 76.995869, -113.211109, -75.099271, -77.224687]
        )
        RT1.DetachAll(CONTF1)
        R1.MoveJ(
            [-72.659792, 46.141127, 66.033817, -115.461384, -79.630306, -66.991431]
        )
        R1.MoveJ(
            [-30.989999, 47.199976, 63.679830, -78.940539, -62.649711, -62.389592]
        )  # home

    def job13():

        CON1.MoveJ([-4720.000000])

    def job14():

        # TVFrame
        R2.MoveJ([35.743958, 46.457908, 79.800060, 65.711867, -65.572361, -99.507682])
        R2.MoveJ([35.761632, 60.636576, 87.774284, 62.722530, -69.048303, -91.726177])
        RT2.AttachClosest("TVFrame1", 1000)
        R2.MoveJ([35.743958, 46.457908, 79.800060, 65.711867, -65.572361, -99.507682])
        R2.MoveJ(
            [-30.989999, 47.199976, 63.679830, -78.940539, -62.649711, -62.389592]
        )  # Home
        R2.MoveJ(
            [-103.268457, 51.204353, 60.557188, -91.436632, -68.246066, -55.018430]
        )  # CON3
        RT2.DetachAll(CONTF4)
        TVpart3_1.setPose(
            Mat(
                [[-1, 0, 0, -2612.1], [0, -1, 0, 1548.85], [0, 0, 1, 120], [0, 0, 0, 1]]
            )
        )

        R2.MoveJ(
            [-103.301352, 38.205475, 48.252689, -96.276467, -69.096794, -41.836787]
        )
        R2.MoveJ(
            [-30.989999, 47.199976, 63.679830, -78.940539, -62.649711, -62.389592]
        )  # Home

    def job15():
        CON1.setPose(Mat([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]))
        CON4.MoveJ([-570.000000 - 30])
        PCB1_R.MoveJ([0])

        Delta2.MoveJ([-16.069340, -16.069340, -16.069340, 0.000000])  # Home
        # 1
        Delta2.MoveJ([41.787953, 2.182667, 36.729479, 0.000000])
        Delta2_T.AttachClosest("PART01", 1000)
        Delta2.MoveJ([-16.069340, -16.069340, -16.069340, 0.000000])  # Home
        Delta2.MoveJ([74.910849, 54.035348, 3.084373, 0.000000])

        Delta2_T.DetachAll(parent=CONTF4)
        Delta2.MoveJ([-16.069340, -16.069340, -16.069340, 0.000000])  # Home

        # 3
        Delta2.MoveJ([57.296287, 1.719071, 37.002685, 0.000000])
        Delta2_T.AttachClosest("PART03", 1000)
        Delta2.MoveJ([-16.069340, -16.069340, -16.069340, 0.000000])  # Home
        Delta2.MoveJ([53.584290, 65.456617, 1.750716, -90.000000])
        Delta2_T.DetachAll(parent=CONTF4)
        Delta2.MoveJ([-16.069340, -16.069340, -16.069340, 0.000000])  # Home

        # 4
        Delta2.MoveJ([29.680608, 5.251382, 39.120596, 0.000000])
        Delta2_T.AttachClosest("PART04", 1000)
        Delta2.MoveJ([-16.069340, -16.069340, -16.069340, 0.000000])  # Home
        Delta2.MoveJ([31.749433, 82.463026, 16.724142, 90.000000])
        Delta2_T.DetachAll(parent=CONTF4)
        Delta2.MoveJ([-16.069340, -16.069340, -16.069340, 0.000000])  # Home

        # 7
        Delta2.MoveJ([48.000754, -1.234691, 51.056880, 90.000000])
        Delta2_T.AttachClosest("PART07", 1000)
        Delta2.MoveJ([-16.069340, -16.069340, -16.069340, 0.000000])  # Home
        CON4.MoveJ([-670.000000 - 30])  # offset
        Delta2.MoveJ([95.061606, 64.209567, 5.786988, 180.000000])
        Delta2_T.DetachAll(parent=CONTF4)
        Delta2.MoveJ([-16.069340, -16.069340, -16.069340, 0.000000])  # Home

        PCB1_R.MoveJ([-1600.000000])

        CON4.MoveJ([-1159.000000 - 30])
        PCB2_R.MoveJ([0.000000])

        Delta1.MoveJ([41.122926, -22.148757, 55.527916, 0.000000])  # home

        # 2
        Delta1.MoveJ([36.999335, 1.922546, 40.786773, 0.000000])
        Delta1_T.AttachClosest("PART02", 1000)
        Delta1.MoveJ([41.122926, -22.148757, 55.527916, 0.000000])  # home
        Delta1.MoveJ([91.941105, 61.639376, 6.057577, 90.000000])
        Delta1_T.DetachAll(parent=CONTF4)
        Delta1.MoveJ([41.122926, -22.148757, 55.527916, 0.000000])  # home

        # 5
        Delta1.MoveJ([55.261195, 1.179381, 37.072134, 0.000000])
        Delta1_T.AttachClosest("PART05", 1000)
        Delta1.MoveJ([41.122926, -22.148757, 55.527916, 0.000000])  # home
        Delta1.MoveJ([31.023922, 42.705471, 9.755680, 0.000000])
        Delta1_T.DetachAll(parent=CONTF4)
        Delta1.MoveJ([41.122926, -22.148757, 55.527916, 0.000000])  # home

        # 6
        Delta1.MoveJ([55.321491, -2.177679, 51.462912, 90.000000])
        Delta1_T.AttachClosest("PART06", 1000)
        Delta1.MoveJ([41.122926, -22.148757, 55.527916, 0.000000])  # home
        CON4.MoveJ([-1260.000000 - 30])
        Delta1.MoveJ([92.478116, 89.697963, 1.823674, 90.000000])
        Delta1_T.DetachAll(parent=CONTF4)
        Delta1.MoveJ([41.122926, -22.148757, 55.527916, 0.000000])  # home
        PCB2_R.MoveJ([-1600.000000])
        CON4.MoveJ([-3100.000000 - 30])

    def job16(num):
        R3.MoveJ([22.876063, 35.177575, 1.281358, 0.000000, 53.541067, 112.876063])
        R3.MoveJ([22.876063, 41.892735, 5.222647, 0.000000, 42.884618, 112.876063])
        RT3.AttachClosest("TV_Guide", 1000)
        RT3.AttachClosest(TVpart1[num], 1000)
        RT3.AttachClosest(TVpart2[num], 1000)
        RT3.AttachClosest("TVFrame1", 1000)
        RT3.AttachClosest("PART01", 1000)
        RT3.AttachClosest("PART02", 1000)
        RT3.AttachClosest("PART03", 1000)
        RT3.AttachClosest("PART04", 1000)
        RT3.AttachClosest("PART05", 1000)
        RT3.AttachClosest("PART06", 1000)
        RT3.AttachClosest("PART07", 1000)
        R3.MoveJ(
            [22.876063, 35.177575, 1.281358, 0.000000, 53.541067, 112.876063]
        )  # home
        R3.MoveJ(
            [78.319947, 55.162684, -33.365702, 0.000000, 68.203018, 78.319947]
        )  # UP
        R3.MoveJ([78.319947, 66.067041, -28.020351, 0.000000, 51.953311, 78.319947])
        RT3.DetachAll(CONTF3)
        R3.MoveJ(
            [78.319947, 55.162684, -33.365702, 0.000000, 68.203018, 78.319947]
        )  # UP
        CON4.setPose(Mat([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]))
        R3.MoveJ(
            [22.876063, 35.177575, 1.281358, 0.000000, 53.541067, 112.876063]
        )  # home

    def job17():
        TVpart4_1.setVisible(True)
        rack4.MoveJ([0.000000, -2100.000000])
        rack1.MoveJ([-3249.990000, -2100.000000])
        rack1.MoveJ([-1373.860000, -2100.000000])
        rack2.MoveJ([-3250.000000, -2100.000000])
        rack3.MoveJ([-3250.000000, 0.000000])
        rack4.MoveJ([0.000000, 0.000000])
        rack4.MoveJ([-359.320000, 0.000000])

    def job18():
        # pickup
        R1.MoveJ(
            [-30.989999, 47.199976, 63.679830, -78.940539, -62.649711, -62.389592]
        )  # home
        R1.MoveJ([-31.034719, 61.798349, 72.753418, -74.608222, -64.747579, -72.240488])
        RT1.AttachClosest("BackPanel1", 1000)
        R1.MoveJ([-30.913124, 27.035174, 42.085122, -90.649707, -60.609735, -37.912265])
        R1.MoveJ(
            [-72.659792, 46.141127, 66.033817, -115.461384, -79.630306, -66.991431]
        )
        R1.MoveJ(
            [-72.655191, 64.934441, 76.995869, -113.211109, -75.099271, -77.224687]
        )
        RT1.DetachAll(CONTF1)
        R1.MoveJ(
            [-72.659792, 46.141127, 66.033817, -115.461384, -79.630306, -66.991431]
        )
        R1.MoveJ(
            [-30.989999, 47.199976, 63.679830, -78.940539, -62.649711, -62.389592]
        )  # home

    def job19():
        CON1.MoveJ([-4720.000000])

    def job20():
        R2.MoveJ([35.743958, 46.457908, 79.800060, 65.711867, -65.572361, -99.507682])
        R2.MoveJ([35.761632, 60.636576, 87.774284, 62.722530, -69.048303, -91.726177])
        RT2.AttachClosest("BackPanel1", 1000)
        R2.MoveJ([35.743958, 46.457908, 79.800060, 65.711867, -65.572361, -99.507682])
        R2.MoveJ(
            [-30.989999, 47.199976, 63.679830, -78.940539, -62.649711, -62.389592]
        )  # Home

        CON3.MoveJ([-2180.000000])
        R2.MoveJ(
            [-139.526545, 38.778140, 103.512499, -23.887105, -82.318653, -92.489062]
        )
        R2.MoveJ(
            [-139.544029, 56.816785, 114.829137, -23.703273, -92.676485, -97.058371]
        )  # CON3

        RT2.DetachAll(CONTF3)
        TVpart4_1.setPose(
            Mat(
                [[0, -1, 0, -217.52], [1, 0, 0, 437.77], [0, 0, 1, 56.66], [0, 0, 0, 1]]
            )
        )
        R2.MoveJ(
            [-30.989999, 47.199976, 63.679830, -78.940539, -62.649711, -62.389592]
        )  # home
        CON3.MoveJ([0])

    def job21(num):
        # foam
        Foam_R.MoveJ([0])
        Foam_R.MoveJ([-980.000000])
        R3.MoveJ([-126.880599, 19.571226, 34.095805, 0.000000, 36.332969, 53.119401])
        R3.MoveJ([-126.880599, 29.136442, 36.987684, 0.000000, 23.875874, 53.119401])
        RT3.AttachClosest("Foam_D", 1000)
        R3.MoveJ([-126.880599, 19.571226, 34.095805, 0.000000, 36.332969, 53.119401])
        R3.MoveJ(
            [105.352937, 9.706977, 42.863121, -0.000000, 37.429902, 105.352937]
        )  # UP2
        R3.MoveJ([105.352937, 36.300006, 47.995549, -0.000000, 5.704445, 105.352937])
        RT3.DetachAll(CONTF5)
        R3.MoveJ(
            [105.352937, 9.706977, 42.863121, -0.000000, 37.429902, 105.352937]
        )  # UP2

        R3.MoveJ(
            [78.319947, 55.162684, -33.365702, 0.000000, 68.203018, 78.319947]
        )  # UP
        R3.MoveJ([78.894359, 64.820665, -25.299027, -0.000000, 50.478362, 78.894359])
        RT3.AttachClosest(TVpart1[num], 1000)
        RT3.AttachClosest(TVpart2[num], 1000)
        RT3.AttachClosest("TVFrame1", 1000)
        RT3.AttachClosest("PART01", 1000)
        RT3.AttachClosest("PART02", 1000)
        RT3.AttachClosest("PART03", 1000)
        RT3.AttachClosest("PART04", 1000)
        RT3.AttachClosest("PART05", 1000)
        RT3.AttachClosest("PART06", 1000)
        RT3.AttachClosest("PART07", 1000)
        RT3.AttachClosest("BackPanel1", 1000)

        R3.MoveJ(
            [78.319947, 55.162684, -33.365702, 0.000000, 68.203018, 78.319947]
        )  # UP
        R3.MoveJ(
            [105.352937, 9.706977, 42.863121, -0.000000, 37.429902, 105.352937]
        )  # UP2
        R3.MoveJ([108.455634, 34.240454, 47.023948, -0.000000, 8.735598, 108.455634])
        RT3.DetachAll(CONTF5)
        R3.MoveJ(
            [105.352937, 9.706977, 42.863121, -0.000000, 37.429902, 105.352937]
        )  # UP2
        R3.MoveJ(
            [78.319947, 55.162684, -33.365702, 0.000000, 68.203018, 78.319947]
        )  # UP
        R3.MoveJ([-126.880599, 19.571226, 34.095805, 0.000000, 36.332969, 53.119401])
        R3.MoveJ([-126.880599, 32.275494, 37.353082, -0.000000, 20.371424, 53.119401])
        RT3.AttachClosest("Foam_U", 1000)
        R3.MoveJ([-126.880599, 19.571226, 34.095805, 0.000000, 36.332969, 53.119401])
        R3.MoveJ(
            [105.352937, 9.706977, 42.863121, -0.000000, 37.429902, 105.352937]
        )  # UP2
        R3.MoveJ([105.541735, 32.331581, 48.086833, 0.000000, 9.581586, 105.541735])
        RT3.DetachAll(CONTF5)
        R3.MoveJ(
            [105.352937, 9.706977, 42.863121, -0.000000, 37.429902, 105.352937]
        )  # UP2
        R3.MoveJ([22.876063, 35.177575, 1.281358, 0.000000, 53.541067, 112.876063])

        RDK.RunProgram("sub_reset1")
        TVpart1_All[num].setVisible(False)
        TVpart2_All[num].setVisible(False)

        time.sleep(2)

    def job22(num):
        R4.MoveJ([17.378747, 5.193797, -54.186247, -0.000000, -35.813753, 72.621253])
        R4.MoveJ(
            [105.076162, -16.682824, -14.614595, -0.000000, -75.385405, -15.076162]
        )
        # box1
        posb1 = Mat(
            [[0, 1, 0, -288.5], [1, 0, 0, 1071], [0, 0, -1, -570], [0, 0, 0, 1]]
        )
        posb2 = Mat(
            [[0, 1, 0, -288.5], [1, 0, 0, 1071], [0, 0, -1, -470], [0, 0, 0, 1]]
        )
        posb3 = Mat(
            [[0, 1, 0, -288.5], [1, 0, 0, 1071], [0, 0, -1, -370], [0, 0, 0, 1]]
        )
        posb4 = Mat(
            [[0, 1, 0, -288.5], [1, 0, 0, 1071], [0, 0, -1, -270], [0, 0, 0, 1]]
        )
        posb5 = Mat(
            [[0, 1, 0, -288.5], [1, 0, 0, 1071], [0, 0, -1, -170], [0, 0, 0, 1]]
        )
        posb6 = Mat([[0, 1, 0, -288.5], [1, 0, 0, 1071], [0, 0, -1, -70], [0, 0, 0, 1]])
        posb7 = Mat([[0, 1, 0, -288.5], [1, 0, 0, 1071], [0, 0, -1, 30], [0, 0, 0, 1]])
        posb8 = Mat([[0, 1, 0, -288.5], [1, 0, 0, 1071], [0, 0, -1, 130], [0, 0, 0, 1]])
        posb9 = Mat([[0, 1, 0, -288.5], [1, 0, 0, 1071], [0, 0, -1, 230], [0, 0, 0, 1]])
        posb10 = Mat(
            [[0, 1, 0, -288.5], [1, 0, 0, 1071], [0, 0, -1, 330], [0, 0, 0, 1]]
        )
        pos1 = [posb10, posb9, posb8, posb7, posb6, posb5, posb4, posb3, posb2, posb1]
        # R4.MoveJ([105.076162, 15.899465, -70.504430, 0.000000, -19.495570, -15.076162])
        R4.MoveL(pos1[num])

        RT4.AttachClosest(boxo[num], 1000)
        RT4.AttachClosest(boxc[num], 1000)

        ##
        R4.MoveJ(
            [105.076162, -16.682824, -14.614595, -0.000000, -75.385405, -15.076162]
        )
        R4.MoveJ([30.074517, 11.269072, -66.606014, 0.000000, -23.393986, 59.925483])
        CON5.MoveJ([970.000000])
        time.sleep(1)

        RT4.AttachClosest("Foam_U", 1000)
        RT4.AttachClosest("Foam_D", 1000)
        CON5.MoveJ([0])

        R4.MoveL(
            Mat(
                [
                    [0, 1, 0, 951.499994],
                    [1, 0, 0, -169.000003],
                    [0, 0, -1, -314],
                    [0, 0, 0, 1],
                ]
            )
        )

        boxo_All[num].setVisible(False)
        boxc_All[num].setVisible(True)
        Foam1.setVisible(False)
        Foam2.setVisible(False)

        R4.MoveJ(
            [-89.455202, 0.586650, -30.355452, -0.000001, -59.634548, -89.994797]
        )  # Home

        # R4.MoveJ([-89.455202, 27.452216, -67.426585, -0.000002, -22.563415, -89.994796])
        pos_a1 = Mat(
            [[-1, 0, 0, -257.5], [0, 1, 0, -1318], [0, 0, -1, -380], [0, 0, 0, 1]]
        )
        pos_a2 = Mat(
            [[-1, 0, 0, -257.5], [0, 1, 0, -1318], [0, 0, -1, -280], [0, 0, 0, 1]]
        )
        pos_a3 = Mat(
            [[-1, 0, 0, -257.5], [0, 1, 0, -1318], [0, 0, -1, -180], [0, 0, 0, 1]]
        )
        pos_a4 = Mat(
            [[-1, 0, 0, -257.5], [0, 1, 0, -1318], [0, 0, -1, -80], [0, 0, 0, 1]]
        )
        pos_a5 = Mat(
            [[-1, 0, 0, -257.5], [0, 1, 0, -1318], [0, 0, -1, 20], [0, 0, 0, 1]]
        )
        pos_a6 = Mat(
            [[-1, 0, 0, 357.5], [0, 1, 0, -1318], [0, 0, -1, -380], [0, 0, 0, 1]]
        )
        pos_a7 = Mat(
            [[-1, 0, 0, 357.5], [0, 1, 0, -1318], [0, 0, -1, -280], [0, 0, 0, 1]]
        )
        pos_a8 = Mat(
            [[-1, 0, 0, 357.5], [0, 1, 0, -1318], [0, 0, -1, -180], [0, 0, 0, 1]]
        )
        pos_a9 = Mat(
            [[-1, 0, 0, 357.5], [0, 1, 0, -1318], [0, 0, -1, -80], [0, 0, 0, 1]]
        )
        pos_a10 = Mat(
            [[-1, 0, 0, 357.5], [0, 1, 0, -1318], [0, 0, -1, 20], [0, 0, 0, 1]]
        )
        pos2 = [
            pos_a1,
            pos_a6,
            pos_a2,
            pos_a7,
            pos_a3,
            pos_a8,
            pos_a4,
            pos_a9,
            pos_a5,
            pos_a10,
        ]

        R4.MoveL(pos2[num])
        RT4.DetachAll(AGV1_TF)
        R4.MoveJ(
            [-89.455202, 0.586650, -30.355452, -0.000001, -59.634548, -89.994797]
        )  # Home
        RDK.RunProgram("sub_reset2")

    def job23():
        rack1.MoveJ([0.000000, -2100.000000])
        rack2.MoveJ([-1040.000000, -2100.000000])
        rack3.MoveJ([-3250.000000, 0.000000])
        rack3.MoveJ([-3250.000000, -2100.000000])
        rack3.MoveJ([-2130.000000, -2100.000000])
        rack4.MoveJ([-3250.000000, 0.000000])
        rack4.MoveJ([-3250.000000, -2100.000000])
        rack4.MoveJ([-3250.000000, -2100.000000])

    def wait_time(t):
        time.sleep(t)

    def rack_reset():
        rack3.MoveJ([0.000000, 0.000000])
        rack3.MoveJ([0.000000, -2100.000000])
        rack3.MoveJ([-2130.00, -2100.00])

        rack2.MoveJ([0.000000, 0.000000])
        rack2.MoveJ([0.000000, -2100.000000])
        rack2.MoveJ([-1040.00, -2100.00])

        rack1.MoveJ([0.000000, 0.000000])
        rack1.MoveJ([0.000000, -2100.000000])
        rack1.MoveJ([0.00, -2100.00])

    while 1:
        RDK.RunProgram("All_reset")
        wait_time(5)
        for i in range(10):
            job1(i)  # Start rack TVFrontPane pick_up to conveyor1
            job2()  # Run conveyor1
            job3(i)  # Start rack TVScreen pick_up to conveyor1
            job4()  # Run conveyor1
            job5()  # TV_Guid pickup, move to conveyor4
            job6(i)  # TVFrontPane move to conveyor4
            job7()  # Run conveyor1
            job8(i)  # TVScreen move to conveyot4
            job9()  # conveyor1 reset
            job10()  ###########move to rack + wait socket sig1

            job11()  # if sig==1 move to rack
            job12()  # Start TVFrame1 pick_up to conveyor1
            job13()  # Run conveyor1
            job14()  # pickup TVFrame1 to conveyor2
            job15()  # PCB 조립
            job16(i)  # CON3에 놓기 wait socket sig2

            job17()  # 백패널 보이기, Rack 이동
            job18()  # BackPanel pick to con1
            job19()  # Run conveyor1
            job20()  # 조립
            job21(i)  # 폼 조립
            job22(i)
            job23()
            time.sleep(5)
