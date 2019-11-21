import os

duration = 1000

view = GetActiveView()
start_time = view.ViewTime
end_time = start_time + duration


# create an animation cue and load code from a script
anim_cue = PythonAnimationCue()
anim_cue_script = "../lib/veloview-3.6/site-packages/veloview/temporal_animation_cue.py"
current_dir = os.getcwd()
with open(os.path.join(current_dir, anim_cue_script), "r") as fin:
	lines = fin.readlines()
anim_cue.Script = "".join(lines)



animation = GetAnimationScene()
animation.Cues.append(anim_cue)
animation.PlayMode = 'Snap To TimeSteps'

animation.StartTime = start_time
animation.EndTime = end_time

animation.Play()