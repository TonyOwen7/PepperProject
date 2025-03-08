from django.urls import path
from . import views

urlpatterns = [
    path('', views.robot_configuration, name='robot_configuration'),
    path('control/', views.control_page, name='control'),
    path('submit/', views.submit_robot_data, name='submit'),
    path('move/', views.handle_move, name='move'),
    path('destination/', views.handle_guiding, name='handle_guiding'),
    path('question/', views.handle_question, name='handle_question'),
    path('speech/', views.handle_speech, name='handle_speech'),
    path('stop_processes/', views.stop_processes, name='stop_processes'),
]