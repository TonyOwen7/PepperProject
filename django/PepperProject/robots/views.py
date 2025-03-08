from django.shortcuts import render, redirect, get_object_or_404
from django.contrib.auth.decorators import login_required
from .models import Robot
from .forms import RobotForm

@login_required
def user_robots(request):
    # Get the robots belonging to the logged-in user
    robots = Robot.objects.filter(user=request.user)
    return render(request, 'robots/user_robots.html', {'robots': robots})

@login_required
def create_robot(request):
    if request.method == 'POST':
        form = RobotForm(request.POST)
        if form.is_valid():
            robot = form.save(commit=False)
            robot.user = request.user  # Associate the robot with the logged-in user
            robot.save()
            return redirect('/robots')  # Redirect to the user's robots page
    else:
        form = RobotForm()
    return render(request, 'robots/create_robot.html', {'form': form})


@login_required
def edit_robot(request, robot_id):
    # Fetch the robot and ensure it belongs to the logged-in user
    robot = get_object_or_404(Robot, id=robot_id, user=request.user)

    if request.method == 'POST':
        # Populate the form with the submitted data and the existing robot instance
        form = RobotForm(request.POST, instance=robot)
        if form.is_valid():
            form.save()  # Save the updated robot data
            return redirect('user_robots')  # Redirect to the user's robots page
    else:
        # Populate the form with the existing robot data
        form = RobotForm(instance=robot)

    return render(request, 'robots/edit_robot.html', {'form': form, 'robot': robot})

@login_required
def delete_robot(request, robot_id):
    # Fetch the robot and ensure it belongs to the logged-in user
    robot = get_object_or_404(Robot, id=robot_id, user=request.user)
    # Delete the robot
    robot.delete()
    # Redirect to the user's robots page
    return redirect('user_robots')

def control_robot(request, robot_id):
    robot = Robot.objects.get(id=robot_id, user=request.user)  # Ensure the robot belongs to the user
    redirect('/submit/', nao_ip=robot.nao_ip, network_interface=robot.network_interface)
    return redirect('/control/')