# Hard coded answers
eval_examples = [
	{
		"query": "What has happened in this ROS2 log regarding navigation?",
		"answer":  """Based on the provided ROS2 log, the following events have occurred:\
					1. The robot received a goal at timestamp 1701354686 and began navigating to goal number 1.\
					2. The robot reached goal number 1 at timestamp 1701354749.\
					3. Navigation to goal number 2 was started at timestamp 1701354749.\
					4. The robot changed planned path when trying to achieve goal pose number 2 because there was an obstacle.\
					5. The robot executed a new path to achieve goal pose number 3 after reaching goal number 2.\
					6. Navigation to goal number 3 has succeeded at timestamp 1701354882.\
					Based on these events, it appears that the navigation task has progressed as expected. \
					and the Nav2 Behavior Tree nodes have handled unexpected situations and adapting to changing environmental conditions."""
	},
	{
		"query": "How has the navigation task proceeded?",
		"answer":  """From the initial position, the robot navigated to the goal number 1, which was located at (-8.17, -5.15). \
					The navigation was successful, and the robot reached the goal location. \
					After reaching the first goal, the navigation task for goal number 2 started. \
					The robot navigated to the new location (-9.22, -25.83) successfully.\
					Finally, the robot was able to successfully reach the third goal (-1.81, -28.60) and complete the navigation task.\
                    Various subsystems such as Navigation2, RateController, ComputePathToPose, and NavigateWithReplanning have been run to help the robot reach its destination."""
	},
	{
		"query": "Has the robot re-planned an alternative trajectory at any time during navigation?",
		"answer":  """Yes, the robot has re-planned an alternative trajectory at least once during navigation."""
	},
	{
		"query": "Why did the robot replan the route?",
		"answer":  """The robot replanned the route because there was an obstacle in the environment."""
	},
	{
		"query": "How many goals have been reached by the robot?",
		"answer":  "The robot has reached 3 goals."
	},
	{
		"query": "Has the robot completed the navigation task?",
		"answer":  """Yes, the robot has completed the navigation task. \
					The context indicates that the robot has successfully navigated to the goal number 3 \
					and the trajectory has been replanned to achieve the goal pose."""
	},
	{
		"query": "When has the robot ended the navigation task?",
		"answer":  "The robot has ended the navigation task at timestamp 1701354882."
	},
	{
		"query": "Where is the second location or goal located?",
		"answer":  "The second goal is located at (-9.22, -25.83)."
	},
    {
        "query": "What was the linear velocity of the robot when navigating to goal pose number 2?",
		"answer":  "The linear velocity of the robot was 0.26000000000000006."
    },
    {
        "query": "What is Nav2 Behavior Tree's node to determine a viable path from a starting point to a specified target pose or location?",
		"answer":  "ComputePathToPose is the node that determines a viable path from a starting point to a specified target pose or location."
    },
    {
        "query": "Did any node from the Nav2 Behavior Tree fail during the navigation? Describe the sequence of events regarding navigation goals and include information about the goal number 1, the goal number 2 and the goal number 3.",
		"answer":  "None of the nodes in the Nav2 Behavior Tree have failed during the navigation."
    },
    {
        "query": "What is the initial position and orientation of the robot?",
		"answer":  "The initial position of the robot is (0.0, 10.00000000000071) and the orientation (-0.706825181105366,0.7073882691671998)."
    },
    {
        "query": "Did the robot find any obstacle during the navigation?",
		"answer":  "Yes, the robot found an obstacle when trying to achieve goal number 2."
    },
    {
        "query": "What was the linear velocity of the robot after receiving the goal number 1?",
		"answer":  "The linear velocity of the robot was 0.24631578947368427."
    },
    {
        "query": "Have all objectives been successfully achieved or have any been cancelled or aborted?",
		"answer":  "All the goals have been successfully achieved."
    },
    {
        "query": "Which Behavior Tree's nodes have been used during the navigation?",
		"answer":  "During the navigation, there were used Nav2 Behavior Tree's nodes ComputePathToPose, FollowPath, RateController, NavigateRecovery and NavigationWithReplanning."
    },
]