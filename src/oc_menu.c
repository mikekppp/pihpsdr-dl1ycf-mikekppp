/* Copyright (C)
* 2015 - John Melton, G0ORX/N6LYT
* 2025 - Christoph van Wüllen, DL1YCF
*
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <https://www.gnu.org/licenses/>.
*
*/

#include <gtk/gtk.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "band.h"
#include "bandstack.h"
#include "client_server.h"
#include "filter.h"
#include "main.h"
#include "message.h"
#include "new_menu.h"
#include "new_protocol.h"
#include "oc_menu.h"
#include "radio.h"

static GtkWidget *dialog = NULL;

static void cleanup() {
  if (dialog != NULL) {
    GtkWidget *tmp = dialog;
    dialog = NULL;
    gtk_widget_destroy(tmp);
    sub_menu = NULL;
    active_menu  = NO_MENU;
    radio_save_state();
  }
}

static gboolean close_cb () {
  cleanup();
  return TRUE;
}

static void oc_rx_cb(GtkWidget *widget, gpointer data) {
  int b = (GPOINTER_TO_UINT(data)) >> 4;
  int oc = (GPOINTER_TO_UINT(data)) & 0xF;
  BAND *band = band_get_band(b);
  int mask = 0x01 << (oc - 1);

  if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))) {
    band->OCrx |= mask;
  } else {
    band->OCrx &= ~mask;
  }

  if (radio_is_remote) {
    send_band_data(client_socket, b);
  } else {
    schedule_high_priority();
  }
}

static void oc_tx_cb(GtkWidget *widget, gpointer data) {
  int b = (GPOINTER_TO_UINT(data)) >> 4;
  int oc = (GPOINTER_TO_UINT(data)) & 0xF;
  BAND *band = band_get_band(b);
  int mask = 0x01 << (oc - 1);

  if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))) {
    band->OCtx |= mask;
  } else {
    band->OCtx &= ~mask;
  }

  if (radio_is_remote) {
    send_band_data(client_socket, b);
  } else {
    schedule_high_priority();
  }
}

static void oc_tune_cb(GtkWidget *widget, gpointer data) {
  int oc = (GPOINTER_TO_UINT(data)) & 0xF;
  int mask = 0x01 << (oc - 1);

  if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))) {
    OCtune |= mask;
  } else {
    OCtune &= ~mask;
  }

  if (radio_is_remote) {
    send_radiomenu(client_socket);
  } else {
    schedule_high_priority();
  }
}

static void oc_full_tune_time_cb(GtkWidget *widget, gpointer data) {
  OCfull_tune_time = gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(widget));

  if (radio_is_remote) {
    send_radiomenu(client_socket);
  } else {
    schedule_high_priority();
  }
}

static void oc_memory_tune_time_cb(GtkWidget *widget, gpointer data) {
  OCmemory_tune_time = gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(widget));

  if (radio_is_remote) {
    send_radiomenu(client_socket);
  } else {
    schedule_high_priority();
  }
}

void oc_menu(GtkWidget *parent) {
  int i, j;
  GtkWidget *lbl;
  GtkRequisition min;
  GtkRequisition nat;
  int width, height;
  dialog = gtk_dialog_new();
  gtk_window_set_transient_for(GTK_WINDOW(dialog), GTK_WINDOW(parent));
  GtkWidget *headerbar = gtk_header_bar_new();
  gtk_window_set_titlebar(GTK_WINDOW(dialog), headerbar);
  gtk_header_bar_set_show_close_button(GTK_HEADER_BAR(headerbar), TRUE);
  gtk_header_bar_set_title(GTK_HEADER_BAR(headerbar), "piHPSDR - Open Collector Output");
  g_signal_connect (dialog, "delete_event", G_CALLBACK (close_cb), NULL);
  g_signal_connect (dialog, "destroy", G_CALLBACK (close_cb), NULL);
  GtkWidget *content = gtk_dialog_get_content_area(GTK_DIALOG(dialog));
  GtkWidget *sw = gtk_scrolled_window_new (NULL, NULL);
  //gtk_widget_set_size_request(sw, 600, 400);
  gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(sw), GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC);
  GtkWidget *viewport = gtk_viewport_new(NULL, NULL);
  GtkWidget *grid = gtk_grid_new();
  gtk_grid_set_column_spacing (GTK_GRID(grid), 10);
  GtkWidget *close_b = gtk_button_new_with_label("Close");
  gtk_widget_set_name(close_b, "close_button");
  g_signal_connect (close_b, "button-press-event", G_CALLBACK(close_cb), NULL);
  gtk_grid_attach(GTK_GRID(grid), close_b, 0, 0, 4, 1);
  lbl = gtk_label_new("Band");
  gtk_widget_set_name(lbl, "boldlabel");
  gtk_grid_attach(GTK_GRID(grid), lbl, 0, 1, 1, 1);
  lbl = gtk_label_new("Rx");
  gtk_widget_set_name(lbl, "boldlabel");
  gtk_grid_attach(GTK_GRID(grid), lbl, 4, 1, 1, 1);
  lbl = gtk_label_new("Tx");
  gtk_widget_set_name(lbl, "boldlabel");
  gtk_grid_attach(GTK_GRID(grid), lbl, 11, 1, 1, 1);
  lbl = gtk_label_new("TuneBits");
  gtk_grid_attach(GTK_GRID(grid), lbl, 15, 1, 1, 1);
  gtk_widget_set_name(lbl, "boldlabel");
  gtk_widget_set_halign(lbl, GTK_ALIGN_CENTER);
  lbl = gtk_label_new("(ORed with TX)");
  gtk_grid_attach(GTK_GRID(grid), lbl, 15, 2, 1, 1);
  gtk_widget_set_name(lbl, "boldlabel");
  gtk_widget_set_halign(lbl, GTK_ALIGN_CENTER);

  for (i = 1; i < 8; i++) {
    char oc_id[16];
    snprintf(oc_id, sizeof(oc_id), "%d", i);
    lbl = gtk_label_new(oc_id);
    gtk_widget_set_name(lbl, "boldlabel");
    gtk_grid_attach(GTK_GRID(grid), lbl, i, 2, 1, 1);
    lbl = gtk_label_new(oc_id);
    gtk_widget_set_name(lbl, "boldlabel");
    gtk_grid_attach(GTK_GRID(grid), lbl, i + 7, 2, 1, 1);
  }

  int bands = radio_max_band();
  int row = 3;
  //
  // fused loop. i runs over the following values:
  // bandGen, 0 ... bands, BANDS ... BANDS+XVTRS-1
  // XVTR bands not-yet-assigned have an empty title
  // and are filtered out
  //
  i = bandGen;

  for (;;) {
    const BAND *band = band_get_band(i);

    if (strlen(band->title) > 0) {
      lbl = gtk_label_new(band->title);
      gtk_widget_set_name(lbl, "boldlabel");
      gtk_grid_attach(GTK_GRID(grid), lbl, 0, row, 1, 1);
      int mask;

      for (j = 1; j < 8; j++) {
        mask = 0x01 << (j - 1);
        GtkWidget *oc_rx_b = gtk_check_button_new();
        gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (oc_rx_b), (band->OCrx & mask) == mask);
        gtk_grid_attach(GTK_GRID(grid), oc_rx_b, j, row, 1, 1);
        g_signal_connect(oc_rx_b, "toggled", G_CALLBACK(oc_rx_cb), GINT_TO_POINTER(j + (i << 4)));
        GtkWidget *oc_tx_b = gtk_check_button_new();
        gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (oc_tx_b), (band->OCtx & mask) == mask);
        gtk_grid_attach(GTK_GRID(grid), oc_tx_b, j + 7, row, 1, 1);
        g_signal_connect(oc_tx_b, "toggled", G_CALLBACK(oc_tx_cb), GINT_TO_POINTER(j + (i << 4)));
      }

      row++;
    }

    // update "loop index"
    if (i == bandGen) {
      i = 0;
    } else if (i == bands) {
      i = BANDS;
    } else {
      i++;
    }

    if (i >= BANDS + XVTRS) { break; }
  }

  int mask;

  for (j = 1; j < 8; j++) {
    char oc_id[16];
    snprintf(oc_id, sizeof(oc_id), "%d", j);
    mask = 0x01 << (j - 1);
    GtkWidget *oc_tune_b = gtk_check_button_new_with_label(oc_id);
    gtk_widget_set_name(oc_tune_b, "boldlabel");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (oc_tune_b), (OCtune & mask) == mask);
    gtk_grid_attach(GTK_GRID(grid), oc_tune_b, 15, j + 2, 1, 1);
    gtk_widget_set_halign(oc_tune_b, GTK_ALIGN_CENTER);
    g_signal_connect(oc_tune_b, "toggled", G_CALLBACK(oc_tune_cb), GINT_TO_POINTER(j));
  }

  j = 10;
  lbl = gtk_label_new("Full Tune(ms):");
  gtk_widget_set_name(lbl, "boldlabel");
  gtk_grid_attach(GTK_GRID(grid), lbl, 15, j, 1, 1);
  j++;
  GtkWidget *oc_full_tune_time_b = gtk_spin_button_new_with_range(750.0, 9950.0, 50.0);
  gtk_spin_button_set_value(GTK_SPIN_BUTTON(oc_full_tune_time_b), (double)OCfull_tune_time);
  gtk_grid_attach(GTK_GRID(grid), oc_full_tune_time_b, 15, j, 1, 2);
  g_signal_connect(oc_full_tune_time_b, "value_changed", G_CALLBACK(oc_full_tune_time_cb), NULL);
  j++;
  j++;
  lbl = gtk_label_new("Memory Tune(ms):");
  gtk_widget_set_name(lbl, "boldlabel");
  gtk_grid_attach(GTK_GRID(grid), lbl, 15, j, 1, 1);
  j++;
  GtkWidget *oc_memory_tune_time_b = gtk_spin_button_new_with_range(250.0, 9950.0, 50.0);
  gtk_spin_button_set_value(GTK_SPIN_BUTTON(oc_memory_tune_time_b), (double)OCmemory_tune_time);
  gtk_grid_attach(GTK_GRID(grid), oc_memory_tune_time_b, 15, j, 1, 2);
  g_signal_connect(oc_memory_tune_time_b, "value_changed", G_CALLBACK(oc_memory_tune_time_cb), NULL);
  gtk_container_add(GTK_CONTAINER(viewport), grid);
  gtk_container_add(GTK_CONTAINER(sw), viewport);
  gtk_container_add(GTK_CONTAINER(content), sw);
  gtk_widget_show_all(dialog);
  sub_menu = dialog;
  //
  // Look how large the scrolled window is! If it is too large for the
  // screen, shrink it. But for large screens we can avoid scrolling by
  // making the dialog just large enough
  //
  gtk_widget_get_preferred_size(viewport, &min, &nat);
  width  = nat.width;
  height = nat.height;

  if (nat.height > display_height[display_size]) {
    height = display_height[display_size] - 50;
    width += 25;  // This accounts for the scroll bar
  }

  gtk_widget_set_size_request(viewport, nat.width, height);
  gtk_widget_set_size_request(sw, width, height);
}

