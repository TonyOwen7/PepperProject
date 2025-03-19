from django.urls import path
from . import views

urlpatterns = [
    path('user_speeches/', views.speech_list, name='user_speeches'),
    path('create/', views.create_speech, name='create_speech'),
    path('edit/<int:speech_id>/', views.edit_speech, name='edit_speech'),
    path('delete/<int:speech_id>/', views.delete_speech, name='delete_speech'),
    path('play/<int:speech_id>/', views.play_speech, name='play_speech'),
]